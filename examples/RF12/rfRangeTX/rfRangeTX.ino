// Transmit sketch used to report quality of reception.
// 2011-05-14 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: rfRangeTX.pde 7678 2011-05-14 13:13:30Z jcw $

// Based on a sample implementation by Steve Evans (@tankslappa).

#include <Ports.h>
#include <RF12.h>

byte outData, pending;
MilliTimer sendTimer;

void setup() {
  rf12_initialize('T', RF12_868MHZ, 88);
}

void loop() {
  rf12_recvDone();

  if (sendTimer.poll(100)) {
    ++outData;
    pending = 1;
  }

  if (pending && rf12_canSend()) {
    rf12_sendStart(0, &outData, sizeof outData);
    pending = 0;
  }
}
