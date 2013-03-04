/// @dir jouleTest
/// Send out a radio packet every minute, consuming as little power as possible.
// 2010-08-29 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

static byte payload[8];

// boilerplate for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void setup() {
#if defined(__AVR_ATtiny84__)
  cli();
  CLKPR = bit(CLKPCE);
  CLKPR = 0; // div 1, i.e. speed up to 8 MHz
  sei();
#endif
  // let the power rise a bit more and let the RFM12B settle
  Sleepy::loseSomeTime(32);
  // finally, do a full init of the RFM12B
  rf12_initialize(17, RF12_868MHZ, 5);
}

void loop() {
  ++payload[0];
  
  rf12_sendNow(0, &payload, sizeof payload);
  rf12_sendWait(3); // mode 3 requires 258 CK startup fuses
  
  rf12_sleep(RF12_SLEEP);
  Sleepy::loseSomeTime(10000);
  rf12_sleep(RF12_WAKEUP);
}
