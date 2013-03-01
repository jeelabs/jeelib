/// @dir test1
/// Simple node-alive check, sends out one test packet per second.
// 2013-02-02 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

const byte LED = 9;
byte counter;

// turn the on-board LED on or off
static void led (bool on) {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, on ? 0 : 1); // inverted logic
}

void setup () {
  // this is node 1 in net group 100 on the 868 MHz band
  rf12_initialize(1, RF12_868MHZ, 100);
}
  
void loop () {
  led(true);

  // actual packet send: broadcast to all, current counter, 1 byte long
  rf12_sendNow(0, &counter, 1);;
  rf12_sendWait(1);

  led(false);

  // increment the counter (it'll wrap from 255 to 0)
  ++counter;
  // let one second pass before sending out another packet
  delay(1000);
}
