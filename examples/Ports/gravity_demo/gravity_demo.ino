/// @dir gravity_demo
/// Demo of the Gravity Plug, based on GravityPlug class in the Ports library.
// 2010-03-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myBus (1);
GravityPlug sensor (myBus);
MilliTimer measureTimer;

struct {
  int axes[3];
  int temp;
} payload;

void setup () {
    Serial.begin(57600);
    Serial.println("\n[gravity_demo]");

    if (sensor.isPresent()) {
      Serial.print("sensor version ");
      sensor.send();
      sensor.write(0x01);
      sensor.receive();
      Serial.println(sensor.read(1), HEX);
      sensor.stop();
    }

    rf12_initialize(7, RF12_868MHZ, 42);
    sensor.begin();
}

void loop () {
    if (measureTimer.poll(1000)) {
        memcpy(payload.axes, sensor.getAxes(), sizeof payload.axes);
        payload.temp = sensor.temperature();

        rf12_sendNow(0, &payload, sizeof payload);

        Serial.print("GRAV ");
        Serial.print(payload.axes[0]);
        Serial.print(' ');
        Serial.print(payload.axes[1]);
        Serial.print(' ');
        Serial.print(payload.axes[2]);
        Serial.print(' ');
        Serial.println(payload.temp);
    }
}
