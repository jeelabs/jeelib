/// @dir test
/// Simple node-alive check, sends out one test packet per second.
// 2013-02-02 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

byte counter;

void setup () {
  // this is node 1 in net group 100 on the 868 MHz band
  rf12_initialize(1, RF12_868MHZ, 100);
}
  
void loop () {
  // standard idiom for waiting until we can send a new packet
  while (!rf12_canSend())
    rf12_recvDone();
  // actual packet send: broadcast to all, current counter, 1 byte long
  rf12_sendStart(0, &counter, 1);

  // next time, the counter will be one higher (it'll wrap from 255 to 0)
  ++counter;
  // let one second pass before sending out the next packet
  delay(1000);
}
