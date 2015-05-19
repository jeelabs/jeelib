/// @dir rf12compat
/// Test for making the RFM12 modules compatible with RFM69 in native mode.
// 2015-05-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

void setup() {
  Serial.begin(57600);
  Serial.println("\n[rf12compat]");
  rf12_initialize(63, RF12_868MHZ, 42, 1720); // 868.6 MHz for testing
}

void loop() {
  if (rf12_recvDone()) {
    Serial.print("OK dst: ");
    Serial.print(rf12_dst, HEX);
    Serial.print(" hdr: ");
    Serial.print(rf12_hdr, HEX);
    Serial.print(' ');
    // include the two CRC bytes at the end
    for (int i = 0; i < rf12_len + 2; ++i) {
      Serial.print(rf12_data[i] >> 4, HEX);
      Serial.print(rf12_data[i] & 0xF, HEX);
    }
    Serial.print(" crc: ");
    Serial.println(rf12_crc);
  }
}
