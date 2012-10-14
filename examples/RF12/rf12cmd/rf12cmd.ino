/// @dir rf12cmd
/// Command-based central node for RF12 packets.
// 2012-10-14 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <EmBencode.h>

#define RF12CMD_VERSION 1
#define BAND 868
#define GROUP 5

char embuf [100];
EmBdecode decoder (embuf, sizeof embuf);
EmBencode encoder;

void EmBencode::PushChar (char ch) {
  Serial.write(ch);
}

static void pushGreeting () {
  encoder.startList();
  encoder.push("hi");
  encoder.push("rf12cmd");
  encoder.push(RF12CMD_VERSION);
  encoder.push(sizeof embuf);
  encoder.endList();
}

static void pushBytes (uint8_t hdr, const void* ptr, uint8_t len) {
  encoder.startList();
  encoder.push("rx");
  encoder.push(BAND);
  encoder.push(GROUP);
  encoder.push(hdr);
  encoder.push(ptr, len);
  encoder.endList();
}

void setup () {
    Serial.begin(115200);
    pushGreeting();

    uint8_t band = BAND == 433 ? RF12_433MHZ :
                    BAND == 868 ? RF12_868MHZ : RF12_915MHZ;
    rf12_initialize(1, band, GROUP);
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0)
    pushBytes(rf12_hdr, (const void*) rf12_data, rf12_len);
}
