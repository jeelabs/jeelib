/// @dir dht_demo
/// Demo readout of the DHT11 or DHT22 temperature/humidity sensors.
/// @see http://jeelabs.org/2012/04/14/reading-out-dhtxx-sensors/
// 2012-04-02 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

DHTxx dht (5); // connected to DIO2

void setup () {
  Serial.begin(57600);
  Serial.println("\n[dht_demo]");
}

void loop () {
  int t, h;
  if (dht.reading(t, h)) {
    Serial.print("temperature = ");
    Serial.println(t);
    Serial.print("humidity = ");
    Serial.println(h);
    Serial.println();
  }
  delay(3000);
}
