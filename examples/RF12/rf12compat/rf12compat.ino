/// @dir rf12compat
/// Test for making the RFM12 modules compatible with RFM69 in native mode.
// 2015-05-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#define RF12_COMPAT 1
#include <JeeLib.h>

void setup() {
  Serial.begin(57600);
  Serial.println("\n[rf12compat]");
  rf12_initialize(31, RF12_868MHZ, 42, 1720); // 868.6 MHz for testing
}

void loop() {
  if (rf12_recvDone()) {
    Serial.print("OK ");
    for (byte i = 0; i < rf12_len; ++i) {
      Serial.print(rf12_data[i] >> 4, HEX);
      Serial.print(rf12_data[i] & 0xF, HEX);
      Serial.print(' ');
    }
    Serial.print("crc: ");
    Serial.println(rf12_crc);
  }
}
