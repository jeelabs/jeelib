/// @dir heading_demo
/// Demo of the Heading Board based on the HDPM01 (with barometer and compass).
// 2010-03-22 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

HeadingBoard sensor (4);
MilliTimer measureTimer;

void setup () {
    Serial.begin(57600);
    Serial.println("\n[heading_demo]");
    rf12_initialize(7, RF12_868MHZ, 5);
    sensor.begin();
}

void loop () {
    if (measureTimer.poll(1000)) {
        struct { int temp, pres, xaxis, yaxis; } payload;

        sensor.pressure(payload.temp, payload.pres);
        sensor.heading(payload.xaxis, payload.yaxis);
        
        rf12_sendNow(0, &payload, sizeof payload);

        Serial.print("HDPM ");
        Serial.print(payload.temp);
        Serial.print(' ');
        Serial.print(payload.pres);
        Serial.print(' ');
        Serial.print(payload.xaxis);
        Serial.print(' ');
        Serial.println(payload.yaxis);
    }
}
