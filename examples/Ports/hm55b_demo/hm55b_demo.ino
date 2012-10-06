/// @dir hm55b_demo
/// Ports demo, reads out the Parallax HM55B compass module # 29123.
// 2009-02-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

/// Interface to the Parallax HM55B compass module.
class HM55B : private Port {
    Port enable;
    
public:
    HM55B (uint8_t dport, uint8_t eport) : Port (dport), enable (eport) {
        mode(INPUT);            // data
        mode2(OUTPUT);          // clock
        digiWrite2(0);
        enable.mode(OUTPUT);    // enable
        enable.digiWrite(1);
    }
    
    void reset() const {
        enable.digiWrite(0);
        mode(OUTPUT);
        shiftWrite(MSBFIRST, 0x0, 4);
        enable.digiWrite(1);
    }
    
    uint16_t measure(int16_t& x, int16_t& y) const {
        uint8_t status = 0;
        enable.digiWrite(0);
        mode(OUTPUT);
        shiftWrite(MSBFIRST, 0x8, 4);
        for (uint8_t i = 0; i < 100; ++i) {
            enable.digiWrite(1);
            enable.digiWrite(0);
            mode(OUTPUT);
            shiftWrite(MSBFIRST, 0xC, 4);
            mode(INPUT);
            status = shiftRead(MSBFIRST, 4);
            if (status != 0)
                break;
            delay(1);
        }
        if (status == 0xC) {
            x = shiftRead(MSBFIRST, 11);
            y = shiftRead(MSBFIRST, 11);
            x = (x << 5) >> 5; // sign-extend bit 11
            y = (y << 5) >> 5; // sign-extend bit 11
        }
        mode(OUTPUT);
        enable.digiWrite(1);
        return status != 0x0C;
    }
};

HM55B compass (3, 4);   // ports 3 & 4, i.e. pins D6/D7/A2/A3 (A3 unused)

void setup() {
    Serial.begin(57600);
    Serial.print("\n[hm55b_demo]");
    
    compass.reset();
}

void loop() {
    delay(1000);
    
    Serial.print("\nCMPS ");
    
    int16_t x, y;
    uint8_t error = compass.measure(x, y);
    Serial.print(x);
    Serial.print(' ');
    Serial.print(y);
    Serial.print(' ');
    Serial.print(error, DEC);
}
