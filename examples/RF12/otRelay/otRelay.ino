/// @dir otRelay
/// Read out the OpenTherm gateway and relay readings using RFM12B.
/// @see http://jeelabs.org/2012/11/20/opentherm-relay/
/// @see http://jeelabs.org/2012/11/21/reducing-the-payload-size/
// 2012-11-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

// buffer to collect last data received
byte lastChars[9], lastFill;

static bool shouldSend (const byte* payload) {
  return true;
}

static byte hex2bin (const byte* hex) {
  byte result = (hex[0] - (hex[0] <= '9' ? 0 : 7)) << 4;
  result |= (hex[1] - (hex[1] <= '9' ? 0 : 7)) & 0x0F;
  return result;
}

static void sendMessage () {
  byte payload [3];
  payload[0] = hex2bin(lastChars + 3);
  payload[1] = hex2bin(lastChars + 5);
  payload[2] = hex2bin(lastChars + 7);

  if (shouldSend(payload)) {
    while (!rf12_canSend())
      rf12_recvDone();
    rf12_sendStart(0, payload, sizeof payload);
  }
}

static void processMessage () {
  switch (lastChars[0]) {
    case 'T': // from thermostat
    case 'B': // from boiler (heater)
      switch (lastChars[1] & 7) {
        case 1: // Write-Data
        case 4: // Read-Ack
          sendMessage();
      }
  }
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
