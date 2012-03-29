// Demo sketch for the Modern Device Compass Board
// 2012-03-29 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myBus (1);
CompassBoard compass (myBus);

void setup() {
    Serial.begin(57600);
    Serial.println("\n[compass_demo]");
}

void loop() {
    Serial.println(compass.heading());
    delay(500);
}
