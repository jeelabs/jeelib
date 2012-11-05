/// @dir pollee
/// This can run on several nodes, and get data to the central "poller" node.
// 2011-11-23 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// Warning: this test will flood the radio band so nothing else gets through!
//
// To prepare for this test, you need to upload RF12demo to each pollee first,
// and set its node ID, group (77) and band (868 MHz). Node IDs must be in the 
// range 1 .. NUM_NODES, as defined in poller.ino (all nodes must be present).

#include <JeeLib.h>

typedef struct {
  byte node;
  long time;
} Payload;

Payload payload;

void setup () {
  Serial.begin(57600);
  Serial.print("\n[pollee]");
  // use the node ID previously stored in EEPROM by RF12demo
  payload.node = rf12_config();
}

void loop () {
  // wait for an incoming empty packet for us
  if (rf12_recvDone() && rf12_crc == 0 && rf12_len == 0 && RF12_WANTS_ACK) {
    // invent some data to send back
    payload.time = millis();
    // start transmission
    rf12_sendStart(RF12_ACK_REPLY, &payload, sizeof payload);
  }
}
