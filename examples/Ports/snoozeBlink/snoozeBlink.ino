/// @dir snoozeBlink
/// Blink a LED on DIO4 using as little power as possible.
// 2011-12-28 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define LED   4   // DIO1 = PD4 = Arduino digital 4
#define PULSE 48  // how long to keep the LED on, in milliseconds

// boilerplate for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void setup () {
  // configure LED pin, start off (i.e. low)
  bitSet(DDRD, LED);
  // turn the radio off in the most power-efficient manner
  Sleepy::loseSomeTime(32);
  rf12_initialize(17, RF12_868MHZ, 5);
  rf12_sleep(RF12_SLEEP);
  // wait another 2s for the power supply to settle
  Sleepy::loseSomeTime(2000);
}

void loop () {
  // turn LED briefly on, then off for the remaining 90% of the time
  bitSet(PORTD, LED);
  Sleepy::loseSomeTime(PULSE);
  bitClear(PORTD, LED);
  Sleepy::loseSomeTime(9 * PULSE);
}
