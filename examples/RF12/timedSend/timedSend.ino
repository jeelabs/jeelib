// Experiment with time-controlled periodic transmission.
// 2011-06-24 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: timedSend.pde 7735 2011-06-24 18:53:51Z jcw $

#include <Ports.h>
#include <RF12.h>

MilliTimer sendTimer;
byte pending;
word seqnum;

void setup () {
  Serial.begin(57600);
  Serial.println("\n[timedSend]");
  rf12_initialize(25, RF12_868MHZ, 4);
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0 && rf12_len == 2) {
    sendTimer.set(0);
    Serial.print(" #");
    Serial.print(seqnum);
    Serial.print(" start: ");
    Serial.print(rf12_data[0], DEC);
    Serial.print(" recvd: ");
    Serial.println(rf12_data[1], DEC);
  }
  
  if (sendTimer.poll(2096))
    pending = 1;
  
  if (pending && rf12_canSend()) {
    pending = 0;
    rf12_sendStart(RF12_HDR_ACK, "hello!", 6);
    ++seqnum;
  }
}
