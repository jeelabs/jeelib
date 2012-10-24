/// @dir lux_demo
/// Demo of the Lux Plug, based on the LuxPlug class in the Ports library.
// 2010-03-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
    
PortI2C myBus (1);
LuxPlug sensor (myBus, 0x39);
byte highGain;

void setup () {
    Serial.begin(57600);
    Serial.println("\n[lux_demo.2]");
    sensor.begin();
}

void loop () {    
    const word* photoDiodes = sensor.getData();
    Serial.print("LUX ");
    Serial.print(photoDiodes[0]);
    Serial.print(' ');
    Serial.print(photoDiodes[1]);
    Serial.print(' ');
    Serial.print(sensor.calcLux());
    Serial.print(' ');
    Serial.println(highGain);

    // need to wait after changing the gain
    //  see http://talk.jeelabs.net/topic/608
    highGain = ! highGain;
    sensor.setGain(highGain);
    delay(1000);
}
