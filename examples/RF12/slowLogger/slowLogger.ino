/// @dir slowLogger
/// Report 4 analog inputs once every minute over wireless.
// 2013-01-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

word payload [4];
byte counter;

void setup () {
  rf12_initialize(20, RF12_868MHZ, 5);
}

void loop () {
  // for special uses: toggle DIO1 every 64 minutes
  pinMode(4, OUTPUT);
  digitalWrite(4, counter++ & 0x40);
    
  for (byte i = 0; i < 4; ++i) {
    // ignore first ADC reading on this channel, and let the ADC MUX settle
    analogRead(i);
    delay(100);
    // accumulate ADC readings to get a more stable (averaged) result
    // readings will end up in range 0..32767, i.e. pseudo 15-bit for 0..VCC
    payload[i] = 0;
    for (byte n = 0; n < 32; ++n)
      payload[i] += analogRead(i);
  }
  
  // send 8-byte packet out
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &payload, sizeof payload);
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
  
  // go to sleep for approx 60 seconds
  Sleepy::loseSomeTime(60000);
}
