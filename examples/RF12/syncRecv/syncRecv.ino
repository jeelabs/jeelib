/// @dir syncRecv
/// Try to receive periodic transmissions with minimal power consumption.
/// @see http://jeelabs.org/2012/11/02/synchronised-reception/
// 2012-10-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define SEND_ID 9       // wait for packets from node 9

word estimate = 3000;   // packet expected every 3s
word window = 512;      // start with +/- 17 % window, power of 2
byte missed;            // count how often in a row receive failed

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static bool isDesiredPacket () {
  return rf12_recvDone() && rf12_crc == 0 && rf12_hdr == SEND_ID;
}

// since the watchdog timer is inaccurate, we must estimate
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
  // the new estimate shouldn't be too close to the exact time
  estimate = (diff + 2) & ~0x0F; // truncate down to 16 ms multiple

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
  if (estimateWithWatchdog()) {
    // gradually narrow down the window whenever reception succeeds
    if (window >= 32)
      window /= 2;
    missed = 0;
  } else {
    // but widen the window again if reception fails for too long
    if (window <= 512 && ++missed % 10 == 0)
      window *= 4; // increase window x4 at every 10 packets missed
  }
}
