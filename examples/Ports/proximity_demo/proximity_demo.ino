/// @dir proximity_demo
/// Demo of the Proximity Plug, based on ProximityPlug class in Ports library.
// 2010-03-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myBus (4, PortI2C::KHZ100); // max 100 KHz!
ProximityPlug sensor (myBus);

void setup () {
    Serial.begin(57600);
    Serial.println("\n[proximity_demo]");
    sensor.begin();
    
    // sensor.send();
    // sensor.write(ProximityPlug::SINFO);
    // sensor.receive();
    // for (;;) {
    //     byte c = sensor.read(0);
    //     if (c == 0)
    //         break;
    //     if (' ' <= c && c <= '~')
    //         Serial.print(c);
    //     else {
    //         Serial.print('<');
    //         Serial.print(c, HEX);
    //         Serial.print('>');
    //     }
    // }
    // sensor.read(1);
    // sensor.stop();
    // Serial.println();
}

void loop() {
    byte v = sensor.getReg(ProximityPlug::TPSTATUS);
    if (v != 0 && v != 0xFF) {
        Serial.print("PROX ");
        Serial.println(v, HEX);
    }
}