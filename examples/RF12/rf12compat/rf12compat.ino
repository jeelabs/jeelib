/// @dir rf12compat
/// Test for making the RFM12 modules compatible with RFM69 in native mode.
// 2015-05-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// Note: you need to set RF12_COMPAT to 1 in "rf12.h" to compile the sketch.

#include <JeeLib.h>

MilliTimer timer;

void setup() {
  Serial.begin(57600);
  Serial.println("\n[rf12compat]");
  rf12_initialize(63, RF12_868MHZ, 42, 1720); // 868.6 MHz for testing
}

void loop() {
  if (rf12_recvDone()) {
    Serial.print(rf12_crc == 0 ? "OK" : " ?");
    Serial.print(" dst: ");
    Serial.print(rf12_dst, HEX);
    Serial.print(" hdr: ");
    Serial.print(rf12_hdr, HEX);
    Serial.print(' ');
    for (int i = 0; i < rf12_len && i < 66; ++i) {
      Serial.print(rf12_data[i] >> 4, HEX);
      Serial.print(rf12_data[i] & 0xF, HEX);
    }
    Serial.println();
  }
  if (timer.poll(1000))
    rf12_sendNow(0, "abc", 3);
}
