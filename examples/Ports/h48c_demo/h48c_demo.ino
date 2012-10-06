/// @dir h48c_demo
/// Ports demo, reads out the Parallax H48C 3-axis accelerometer # 29123.
// 2009-02-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

/// Interface to the Parallax H48C 3-axis accelerometer.
class H48C : private Port {
    Port enable;
    
public:
    H48C (uint8_t dport, uint8_t eport) : Port (dport), enable (eport) {
        mode(INPUT);            // data
        mode2(OUTPUT);          // clock
        digiWrite2(0);
        enable.mode(OUTPUT);    // enable
        enable.digiWrite(1);
    }
    
    int16_t measure(uint8_t reg) const {
        uint8_t status = 0;
        enable.digiWrite(0);
        mode(OUTPUT);
        shiftWrite(MSBFIRST, 0x18 + reg, 5);
        delay(1);
        mode(INPUT);
        int16_t result = shiftRead(MSBFIRST, 13);
        mode(OUTPUT);
        enable.digiWrite(1);
        return (result << 4) >> 4; // sign-extend bit 11
    }
};

H48C accel (1, 2);   // ports 1 & 2, i.e. pins D4/D5/A0/A1 (A1 unused)

void setup() {
    Serial.begin(57600);
    Serial.print("\n[h48c_demo]");
}

void loop() {
    delay(1000);
    
    Serial.print("\nHACC ");
    
    int16_t v = accel.measure(3);
    Serial.print(v);

    int16_t x = accel.measure(0);
    Serial.print(" X ");
    Serial.print(x - v);

    int16_t y = accel.measure(1);
    Serial.print(" Y ");
    Serial.print(y - v);

    int16_t z = accel.measure(2);
    Serial.print(" Z ");
    Serial.print(z - v);
}
