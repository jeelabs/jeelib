/// @dir lux_poweroff
/// Low power sleeping for the Lux Plug.
// Based on the Lux_demo by <jc@wippler.nl>
// 2012-01-22 <vliegendehuiskat@gmail.com> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myBus (3);
LuxPlug sensor (myBus, 0x39);
byte highGain;

void setup () {
    Serial.begin(57600);
    Serial.println("\n[lux_poweroff.1]");
}

void loop () {
    sensor.begin();
    sensor.setGain(highGain);
    delay(1000); // Wait for proper powerup.
    
    const word* photoDiodes = sensor.getData();
    Serial.print("LUX ");
    Serial.print(photoDiodes[0]);
    Serial.print(' ');
    Serial.print(photoDiodes[1]);
    Serial.print(' ');
    Serial.print(sensor.calcLux());
    Serial.print(' ');
    Serial.println(highGain);
    sensor.poweroff(); // Power off when we've got the data.
    
    highGain = ! highGain;
    delay(6000);
}
