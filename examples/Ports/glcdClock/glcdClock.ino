/// @dir glcdClock
/// Demo clock for the Graphics Board, this version runs on the internal clock.
/// @see http://jeelabs.org/2010/11/22/the-obligatory-clock…/
// 2010-11-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <ST7565.h>
#include <RTClib.h>
#include <Wire.h> // needed to avoid a linker error :(
#include <avr/pgmspace.h>
#include "digits.h"

ST7565 glcd(14, 4, 17, 7);
RTC_Millis rtc;

static void drawDigit(byte x, byte d) {
    const long* digit = digits + 64 * d;
    for (byte i = 0; i < 64; ++i) {
        long mask = pgm_read_dword(digit++);
        for (byte j = 0; j < 28; ++j)
            glcd.setpixel(x + j, i, bitRead(mask, 27-j));
    }
}

static void twoDigits(byte x, byte v) {
    drawDigit(x, v / 10);
    drawDigit(x + 32, v % 10);
}

void setup () {
    rtc.begin(DateTime (__DATE__, __TIME__));
    
    glcd.st7565_init();
    glcd.st7565_command(CMD_DISPLAY_ON);
    glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
    glcd.st7565_set_brightness(0x15);
    glcd.clear();

#if 1    
    pinMode(3, OUTPUT);
    digitalWrite(3, 1); // turn the backlight on
#endif
}

void loop () {
    DateTime now = rtc.now();
    // hours
    twoDigits(0, now.hour());
    // minutes
    twoDigits(69, now.minute());
    // blinking colon
    glcd.fillcircle(63, 24, 2, now.second() & 1);
    glcd.fillcircle(63, 40, 2, now.second() & 1);
    // show it!
    glcd.display();
}
