/// @dir syncRecv
/// Try to receive periodic transmissions with minimal power consumption.
// 2012-10-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define SEND_ID 9      // wait for packets from node 9

word estimate = 3000;  // packet expected every 3s
word window = 512;     // start with a +/- 17 % window

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static bool isDesiredPacket () {
  return rf12_recvDone() && rf12_crc == 0 && rf12_hdr == SEND_ID;
}

// the since watchdog timer is inaccurate, tune the estimate first
static bool estimateWithWatchdog () {
  // first wait indefinitely for a packet to come in
  while (!isDesiredPacket())
    ;
  Serial.print("R");
  delayMicroseconds(250);

  // sleep until estimate - window
  word lowEstimate = estimate - window;
  uint32_t start = millis();
  rf12_sleep(RF12_SLEEP);
  Sleepy::loseSomeTime(lowEstimate);
  rf12_sleep(RF12_WAKEUP);

  // using the millis() timer, measure when the next packet comes
  word diff;
  do {
    diff = millis() - start;
    if (diff > estimate + window)
      return false;
  } while (!isDesiredPacket());

  int offset = window - (diff - lowEstimate);
  estimate = diff & ~0x0F; // truncate down to a 16 ms multiple

  Serial.print(" ok at "); Serial.print(diff);
  Serial.print(", window = "); Serial.print(window);
  Serial.print(", offset = "); Serial.print(offset);
  Serial.print(", new estimate = "); Serial.println(estimate);
  return true;
}

void setup () {
  Serial.begin(57600);
  Serial.println("\n[syncRecv]");
  rf12_initialize(1, RF12_868MHZ, 5); // never sends anything
}

void loop () {
  // gradually improve the estimate each time reception succeeds
  if (estimateWithWatchdog() && window >= 32)
    window /= 2;
}
