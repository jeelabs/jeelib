/// @dir otRelay
/// Read out the OpenTherm gateway and relay readings using RFM12B.
// 2012-11-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

// buffer to collect last data received
byte lastChars[9], lastFill;

static void processMessage () {
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, lastChars, lastFill);
}
  
void setup () {
  Serial.begin(9600);
  rf12_initialize(14, RF12_868MHZ, 5);
}

void loop () {
  if (Serial.available()) {
    char c = Serial.read();
    if (lastFill < sizeof lastChars) {
      if ('0' <= c && c <= '9' || 'A' <= c && c <= 'Z') {
        // it fits and it's a valid character
        lastChars[lastFill++] = c;
        return;
      }
    } else if (c == '\r')
      processMessage();
    // done, or invalid: clear the lastChars buffer
    lastFill = 0;
  }
}
