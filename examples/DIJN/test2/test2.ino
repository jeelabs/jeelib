/// @dir test2
/// Very simple demo sketch to transmit LDR light readings once a second
// 2013-02-16 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define LDR 0 // the LDR will be connected to AIO1, i.e. analog 0

void setup () {
  // this is node 1 in net group 100 on the 868 MHz band
  rf12_initialize(1, RF12_868MHZ, 100);

  // need to enable the pull-up to get a voltage drop over the LDR
  pinMode(14+LDR, INPUT);
  digitalWrite(14+LDR, 1); // pull-up
}
  
void loop () {
  // measure analog value and convert the 0..1023 result to 255..0
  byte value = 255 - analogRead(LDR) / 4;

  // actual packet send: broadcast to all, current counter, 1 byte long
  rf12_sendNow(0, &value, 1);

  // let one second pass before sending out another packet
  delay(1000);
}
