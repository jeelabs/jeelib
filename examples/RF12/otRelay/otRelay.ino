/// @dir otRelay
/// Read out the OpenTherm gateway and relay readings using RFM12B.
/// @see http://jeelabs.org/2012/11/20/opentherm-relay/
/// @see http://jeelabs.org/2012/11/21/reducing-the-payload-size/
/// @see http://jeelabs.org/2012/11/22/reducing-the-packet-rate/
// 2012-11-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define RESEND_COUNT 10   // send each new value out this often

// buffer to collect last data received
byte lastChars[9], lastFill;

MilliTimer resendTimer;   // fires once a second
byte resendCursor;        // cycles over all the history entries

// the last values received, one entry for each ID 0..255
struct { word value; byte resends; } history [256];

static byte hex2bin (const byte* hex) {
  byte result = (hex[0] - (hex[0] <= '9' ? 0 : 7)) << 4;
  result |= (hex[1] - (hex[1] <= '9' ? 0 : 7)) & 0x0F;
  return result;
}

static void sendId (byte id) {
  if (history[id].resends > 0) {
    --history[id].resends;

    byte payload [3];
    payload[0] = id;
    payload[1] = history[id].value;       // litle-endian
    payload[2] = history[id].value >> 8;

    rf12_sendNow(0, payload, sizeof payload);
  }
}

static void sendIfChanged () {
  byte id = hex2bin(lastChars+3);
  word newval = (hex2bin(lastChars+5) << 8) + hex2bin(lastChars+7);

  history[id].resends = RESEND_COUNT;
  if (newval != history[id].value) {
    history[id].value = newval;
    sendId(id);
  }
}

static void processMessage () {
  switch (lastChars[0]) {
    case 'T': // from thermostat
    case 'B': // from boiler (heater)
      switch (lastChars[1] & 7) {
        case 1: // Write-Data
        case 4: // Read-Ack
          sendIfChanged();
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

  // periodically check whether the next id needs to be resent
  if (resendTimer.poll(1000))
    sendId(++resendCursor);
}
