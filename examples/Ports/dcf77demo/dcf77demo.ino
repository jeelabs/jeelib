/// @dir  dcf77demo
/// Port demo, DCF77 radio signal decoder.
/// @see http://jeelabs.org/2009/03/06/dcf77-clock-reception/
// 2009-02-26 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/pgmspace.h>

Port DCF77 (1);

static uint16_t dcfWidth;
static uint8_t dcfLevels, dcfBits, dcfParity;
static uint8_t dcfValue[8];
static uint8_t year, month, day, hour, minute, dst;

static prog_uint8_t daysInMonth[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint8_t y, uint8_t m, uint8_t d) {
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y - 1) / 4;
}

static uint32_t unixTime(uint16_t days, uint8_t h, uint8_t m, uint8_t s, uint8_t dst) {
    uint32_t secs = 946681200L; // 2000/01/01 00:00:00 CET
    return secs + ((((days * 24L + h + (dst ? -1 : 0)) * 60) + m) * 60) + s;
}

static uint8_t dcfExtract(uint8_t pos, uint8_t len) {
    uint16_t *p = (uint16_t*) (dcfValue + (pos >> 3));
    uint8_t val = (*p >> (pos & 7)) & ((1 << len) - 1);
    return val - (val / 16) * 6; // bcd -> dec
}

static uint8_t dcfMinute() {
    year = dcfExtract(50, 8);
    month = dcfExtract(45, 5);
    day = dcfExtract(36, 6);
    hour = dcfExtract(29, 6);
    minute = dcfExtract(21, 7);
    dst = dcfExtract(17, 1);
    return 1 <= year && year <= 99 && 1 <= month && month <= 12 &&
            1 <= day && day <= 31 && hour <= 23 && minute <= 59;
}

static uint8_t dcfPoll(uint8_t signal) {
    uint8_t ok = 0;
    static uint32_t last;
    if (millis() != last) {
        last = millis();

        // track signal levels using an 8-bit shift register
        dcfLevels = (dcfLevels << 1) | signal;
        if (dcfLevels == 0x07F) {
            if (dcfWidth > 1000) {
                if (dcfBits == 59)
                    ok = dcfMinute();
                memset(dcfValue, 0, sizeof dcfValue);
                dcfBits = 0;
            }
            dcfWidth = 0;
        } else if (dcfLevels == 0xFE) {
            if (0) {
                Serial.print("dcf width ");
                Serial.println((int) dcfWidth);
            }
            if (dcfWidth >= 144) {
                dcfValue[dcfBits>>3] |= _BV(dcfBits & 7);
                dcfParity ^= 1;
            }
            switch (++dcfBits) {
                case 15: dcfParity = 0;
                case 29: case 36: case 59: if (dcfParity) dcfBits = 0;
            }
            dcfWidth = 0;
        }
        ++dcfWidth;
    }
    return ok;
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[dcf77demo]");

    DCF77.mode(INPUT);
    DCF77.digiWrite(1); // pull-up
}

void loop() {
    if (dcfPoll(DCF77.digiRead())) {
        Serial.print("DCF ");
        // char buf[30];
        // sprintf(buf, "20%02d/%02d/%02d %02d:%02d %d %lu",
        //                 year, month, day, hour, minute, dst,
        //                 unixTime(date2days(year, month, day),
        //                             hour, minute, 0, 0));
        // Serial.println(buf);
        Serial.print((int) dst);
        Serial.print(' ');
        if (1) {
            Serial.print(2000 + year);
            Serial.print(' ');
            Serial.print((int) month);
            Serial.print(' ');
            Serial.print((int) day);
            Serial.print(' ');
            Serial.print((int) hour);
            Serial.print(' ');
            Serial.print((int) minute);
            Serial.print(' ');
        }
        Serial.println(unixTime(date2days(year, month, day),
                                hour, minute, 0, 0));
    }
}
