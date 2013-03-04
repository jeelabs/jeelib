/// @dir poller
/// Polls a number of "pollee" nodes as fast as possible to get data from them.
// 2011-11-23 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// Warning: this test will flood the radio band so nothing else gets through!
//
// Maximum rates will only be achieved if all nodes 1 .. NUM_NODES are powered
// up, properly configured, and pre-loaded with the pollee.ino sketch.

#include <JeeLib.h>

const byte NUM_NODES = 3; // poll using node ID from 1 to NUM_NODES 

typedef struct {
  byte node;
  long time;
} Payload;

byte nextId;
MilliTimer timer;

void setup () {
  Serial.begin(57600);
  Serial.println("\n[poller]");
  rf12_initialize(1, RF12_868MHZ, 77);
}

void loop () {
  // switch to next node
  if (++nextId > NUM_NODES)
    nextId = 1;
  // send an empty packet to one specific pollee
  rf12_sendNow(RF12_HDR_ACK | RF12_HDR_DST | nextId, 0, 0);
  // wait up to 10 milliseconds for a reply
  timer.set(10);
  while (!timer.poll())
    if (rf12_recvDone() && rf12_crc == 0 && rf12_len == sizeof (Payload)) {
      // got a good ACK packet, print out its contents
      const Payload* p = (const Payload*) rf12_data;
      Serial.print((word) p->node);
      Serial.print(": ");
      Serial.println(p->time);
      break;
    }
}
