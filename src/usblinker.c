/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"
#include "usblinker.h"
#include "drv_pwm.h"


// Copy-pasted from simonk
// All transmissions have a leader of 23 1-bits followed by 1 0-bit.
// Bit encoding starts at the least significant bit and is 8 bits wide.
// 1-bits are encoded as 64.0us high, 72.8us low (135.8us total).
// 0-bits are encoded as 27.8us high, 34.5us low, 34.4us high, 37.9 low
// (134.6us total)
// End of encoding adds 34.0us high, then return to input mode.
// The last 0-bit low time is 32.6us instead of 37.9us, for some reason.

const escHardware_t escHardware[6] = {
// Figure out esc clocks and pins, extrapolated from timer.c
// Periphs could be pulled progmatically... but I'll leave that for another exercise
  { GPIOA, GPIO_Pin_8 },
  { GPIOA, GPIO_Pin_11 },
  { GPIOB, GPIO_Pin_6 },
  { GPIOB, GPIO_Pin_7 },
  { GPIOB, GPIO_Pin_8 },
  { GPIOB, GPIO_Pin_9 }
};



#define BIT_DELAY_HALF 34
#define BIT_DELAY 68

volatile uint8_t serialBuffer[255];

static bool isEmpty(const char *string)
{
    return *string == '\0';
}

void gpio_config_out(uint16_t escIndex)
{
    gpio_config_t gpcfg;

    gpcfg.mode = Mode_Out_PP;
    gpcfg.pin = escHardware[escIndex].pinpos;
    gpcfg.speed = Speed_2MHz;

    gpioInit(escHardware[escIndex].gpio, &gpcfg);
}

void gpio_config_in(uint16_t escIndex)
{
    gpio_config_t gpcfg;

    gpcfg.mode = Mode_IPU;
    gpcfg.pin = escHardware[escIndex].pinpos;
    gpcfg.speed = Speed_2MHz;

    gpioInit(escHardware[escIndex].gpio, &gpcfg);
}

void sendDigital1(uint16_t escIndex)
{
    digitalHi(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY);
    digitalLo(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY);
}

void sendDigital0(uint16_t escIndex)
{
    digitalHi(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY_HALF);
    digitalLo(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY_HALF);
    digitalHi(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY_HALF);
    digitalLo(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY_HALF);
}

void sendByte(uint8_t byte,uint16_t escIndex)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        if(byte & (1 << i))
        {
            sendDigital1(escIndex);
        } else {
            sendDigital0(escIndex);
        }
    }
}

void sendBuf(uint8_t txlen,uint16_t escIndex)
{
    gpio_config_out(escIndex);

    // send intro message
    for(uint8_t i = 0; i < 23; i++)
    {
        sendDigital1(escIndex);
    }
    sendDigital0(escIndex);

    for(uint8_t i = 0; i < txlen; i++)
    {
        sendByte(serialBuffer[i],escIndex);
    }

    // send trailing message
    digitalHi(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);
    delayMicroseconds(BIT_DELAY_HALF);

    gpio_config_in(escIndex);
}

int8_t readBit(uint32_t bitPeriod,uint16_t escIndex)
{
    uint32_t startTime = micros();
    while(digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait to go low
        if (micros() > startTime + 250)
            return -1;
    while(!digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait to go high
        if (micros() > startTime + 250)
            return -1;
    uint32_t endTime = micros();

    if((endTime - startTime) < (bitPeriod / 1.5)) // short pulses
    {
        while(digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait for second half of bit
            if (micros() > startTime + 250)
                return -1;
        while(!digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos))
            if (micros() > startTime + 250)
                return -1;
        return 0;
    }
    return 1;
}

void USBLinker(char *cmdline)
{


    int escIndex;

    if (isEmpty(cmdline)) {
           printf("Please specify a ouput channel. e.g. `usblinker 2` to connect to motor 2\r\n");
        return;
    } else {
        escIndex = atoi(cmdline);
        if (escIndex >= 0 && escIndex <= 6) {
            printf("Switching to SimonK mode on motor port %d\r\n", escIndex);
        }
        else {
            printf("Invalid motor port, valid range: 1 to %d\r\n", 6);
        }
    }
    // StopPwmAllMotors();
    pwmShutdownPulsesForAllMotors(6);

    escIndex = escIndex-1;
    LED0_OFF
    LED1_OFF
    gpio_config_in(escIndex);
    serialInit(19200);

    uint16_t lastPin = 0;

    while(1)
    {
        if (serialTotalBytesWaiting(core.mainport))
        {
            LED1_ON;
            uint16_t rxlen = 0;
            uint32_t lastRX = millis();

            do {
                if (serialTotalBytesWaiting(core.mainport))
                {
                    serialBuffer[rxlen++] = serialRead(core.mainport);
                    lastRX = millis();

                    if (rxlen == 255)
                    {
                        while(serialTotalBytesWaiting(core.mainport))
                            serialRead(core.mainport);
                        break;
                    }
                }
            }
            while (millis() < lastRX + 2);

            LED1_OFF;
            sendBuf(rxlen,escIndex);
            lastPin = 1;
        } else {
            // read from pin
            uint16_t curPin = digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos);

            if ((lastPin == 0) && (curPin != 0)) // pin went high from low
            {
                LED0_ON
                // get sync time from header
                volatile uint32_t startTime, endTime, bitPeriod;

                startTime = micros();

                // get starting time at next low-high transition
                while(digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait to go low
                    if (micros() > startTime + 250)
                                break;
                while(!digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait to go high
                    if (micros() > startTime + 250)
                                break;
                startTime = micros();

                // get ending time at next low-high transition
                while(digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait to go low
                    if (micros() > startTime + 250)
                                break;
                while(!digitalIn(escHardware[escIndex].gpio, escHardware[escIndex].pinpos)) // wait to go high
                    if (micros() > startTime + 250)
                                break;
                endTime = micros();

                bitPeriod = endTime - startTime; // doesn't include overflow case

                uint8_t introCount = 0;
                while(readBit(bitPeriod,escIndex) == 1) // exit on last intro bit, which is 0
                {
                    introCount++;
                }
                if (introCount > 10) // decent threshold
                {
                    uint8_t rxlen = 0;
                    int8_t tmp;
                    uint8_t timeout = 0;
                    while(timeout == 0)
                    {
                        for (int8_t i = 0; i < 8; i++)
                        {
                            if (i == 0)
                                serialBuffer[rxlen] = 0; // reset byte for bitwise operations
                            tmp = readBit(bitPeriod,escIndex);
                            if (tmp == -1) // timeout reached
                            {
                                timeout = 1;
                                break;
                            } else {
                                serialBuffer[rxlen] |=  (tmp << i); // LSB first
                                if (i == 7)
                                    rxlen++;
                            }
                        }
                    }

                    for (uint8_t i = 0; i < rxlen; i++)
                    {
                        serialWrite(core.mainport, serialBuffer[i]);
                    }
                }
                LED0_OFF
            }
            lastPin = curPin;
        }
    }
}
