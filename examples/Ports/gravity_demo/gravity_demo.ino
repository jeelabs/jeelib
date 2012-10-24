/// @dir gravity_demo
/// Demo of the Gravity Plug, based on GravityPlug class in the Ports library.
// 2010-03-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myBus (1);
GravityPlug sensor (myBus);
MilliTimer measureTimer;

void setup () {
    Serial.begin(57600);
    Serial.println("\n[gravity_demo]");
    rf12_initialize(7, RF12_868MHZ, 5);
    sensor.begin();
}

void loop () {
    if (measureTimer.poll(1000)) {
        const int* p = sensor.getAxes();

        while (!rf12_canSend())
            rf12_recvDone();
        rf12_sendStart(0, p, 3 * sizeof *p, 2);

        Serial.print("GRAV ");
        Serial.print(p[0]);
        Serial.print(' ');
        Serial.print(p[1]);
        Serial.print(' ');
        Serial.println(p[2]);
    }
}
