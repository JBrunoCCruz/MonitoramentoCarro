// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

/*
 * Taken from https://github.com/adafruit/RTClib
 * and modified for LPC1768 by Neal Horman July 2012
 * Also add support for access to the 56 bytes of
 * user accessible battery backed ram.
 */

#ifndef _DS1307_H_
#define _DS1307_H_

#include "mbed.h"
#include "DateTime.h"

// RTC based on the DS1307 chip connected via I2C
class RtcDs1307
{
public:
    RtcDs1307(I2C &i2c);
    bool adjust(const DateTime& dt);
    bool isRunning();
    DateTime now();
    bool commit();
    uint8_t &operator[](uint8_t i) { return mRam[(i<sizeof(mRam)-1 ? i+1 : 0)]; };
protected:
    I2C &mI2c;
    uint8_t mRam[1+56]; // device register address + 56 bytes
};

#endif