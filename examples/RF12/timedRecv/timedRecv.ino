/// @dir timedRecv
/// Experiment with time-controlled periodic reception.
// 2011-06-24 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/sleep.h>

struct {
  byte start;   // time at which we started listening for a packet
  byte later;   // how long we had to wait for packet to come in
} payload;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void setup () {
  rf12_initialize(26, RF12_868MHZ, 4);
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0) {
    // process incoming data here
    
    if (RF12_WANTS_ACK) {
      payload.later = (byte) millis() - payload.start;
      rf12_sendStart(RF12_ACK_REPLY, &payload, sizeof payload);
      rf12_sendWait(1); // don't power down too soon
    }
    
    // power down for 2 seconds (multiple of 16 ms)
    rf12_sleep(RF12_SLEEP);
    Sleepy::loseSomeTime(2000);
    rf12_sleep(RF12_WAKEUP);
    
    // just woke up, start listening for a packet again
    payload.start = millis();
  } else {
    // switch into idle mode until the next interrupt
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
}
