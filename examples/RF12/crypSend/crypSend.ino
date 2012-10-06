/// @dir crypSend
/// Test encrypted communication, sender side.
// 2010-02-21 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

MilliTimer sendTimer;
byte sendSize;
char payload[] = "ABC 0123456789";

void setup () {
    Serial.begin(57600);
    Serial.println("\n[crypSend]");
    rf12_initialize(1, RF12_868MHZ, 33);
    rf12_encrypt(RF12_EEPROM_EKEY);
}

void loop () {
    rf12_recvDone();
    if (rf12_canSend() && sendTimer.poll(3000)) {
        // send out a new packet every 3 seconds
        Serial.print("  send ");
        Serial.println((int) sendSize);
        // send as broadcast, payload will be encrypted
        rf12_sendStart(0, payload, sendSize + 4);
        sendSize = (sendSize + 1) % 11;
    }
}
