/// @dir homePower
/// Send incoming home power measurements as wireless packets.
/// @see http://jeelabs.org/2012/10/22/sending-out-pulses/
// 2012-10-20 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define LED   5   // DIO2
#define POW1  15  // AIO2
#define POW2  16  // AIO3
#define POW3  6   // DIO3

#define NUMPINS 3

// I/O pin numbers connected to the pulse hardware
byte pins [NUMPINS] = { POW1, POW2, POW3 };

byte states [NUMPINS];      // last 8 pin states, as bits
word counters [NUMPINS];    // pulse counts
long times [NUMPINS];       // time of last pulse
bool pending;               // results waiting to be sent
bool wantToSend;            // true if now is the time to send

MilliTimer blinkTimer, scanTimer, sendTimer;
word blinkPattern;

typedef struct {
  word count;
  word tdiff;
} PayloadItem;

PayloadItem payload [NUMPINS];

// prepare to blink the LED a few times
static void setBlinks (char n) {
  // don't mess up if currently blinking
  if (blinkPattern == 0)
    while (--n >= 0)
      blinkPattern = (blinkPattern << 2) | 1;
}

// "compress" a long into a 16-bit unsigned int
static word compress (unsigned long value) {
  if (value > 5000000)
    return 65000;
  if (value > 60000)
    return 60000 + value / 1000;
  return value;
}

// a pulse has been detected, deal with it
static void gotPulse (byte index) {
  setBlinks(index + 1);
  pending = true;
  // measure time since last pulse and save the info
  unsigned long now = millis();
  payload[index].count = ++counters[index];
  payload[index].tdiff = compress(now - times[index]);
  times[index] = now;
}

void setup () {
  Serial.begin(57600);
  Serial.print("\n[homePower]");
  pinMode(LED, OUTPUT);
  setBlinks(2);
  rf12_initialize(9, RF12_868MHZ, 5);
}

void loop () {
  rf12_recvDone(); // ignore incoming packets

  // only want to send every 3 s, if there are changes
  if (sendTimer.poll(3000) && pending) {
    wantToSend = true;
    pending = false;
  }

  // only send when it's actually allowed to do so
  if (wantToSend && rf12_canSend()) {
    rf12_sendStart(0, payload, sizeof payload);
    wantToSend = false;
  }

  if (scanTimer.poll(1))
    for (byte i = 0; i < sizeof pins; ++i) {
      // track the last 8 pin states, scanned every millisecond
      // if state is 0x01, then we saw 7 times 0 followed by a 1
      states[i] <<= 1;
      states[i] |= digitalRead(pins[i]);
      if (states[i] == 0x01)
        gotPulse(i);
    }

  if (blinkTimer.poll(100)) {
    digitalWrite(LED, blinkPattern & 1);
    blinkPattern >>= 1;
  }
}
