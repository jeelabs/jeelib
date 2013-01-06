/// @dir ookScope2
/// Examine the pulse patterns coming from an OOK receiver (see also peekrf).
// 2010-04-10 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// see http://jeelabs.org/2010/04/13/an-ook-scope/
// changed to use pin-change interrupts i.s.o. the analog comparator

#define OOK_PIN   2   // this is the input pin with the signal to be analyzed
#define OOK_POWER 12  // define this is a pin needs to be set to power up

volatile byte next, fill, buf[256];
static word last;

#if OOK_PIN >= 14
#define VECT PCINT1_vect
#elif OOK_PIN >= 8
#define VECT PCINT0_vect
#else
#define VECT PCINT2_vect
#endif

ISR(VECT) {
  // width is the pulse length in usecs, for either polarity
  word width = (micros() >> 2) - last;
  last += width;

  if (width <= 5)
    return; // ignore pulses <= 20 µs
      
  if (width >= 128) {
    width = (width >> 1) + 64;
    if (width >= 192) {
      width = (width >> 1) + 96;
      if (width >= 224) {
        width = (width >> 1) + 112;
        if (width >= 240) {
          width = (width >> 1) + 120;
          if (width >= 248) {
            width = (width >> 1) + 124;
            if (width >= 252) {
              width = (width >> 1) + 126;
              if (width > 255)
                width = 255;
            }
          }
        }
      }
    }
  }
  buf[fill++] = width;
}

static void setupPinChangeInterrupt () {
  pinMode(OOK_PIN, INPUT);
  digitalWrite(OOK_PIN, 1);   // pull-up
#if OOK_PIN >= 14
  bitSet(PCMSK1, OOK_PIN - 14);
  bitSet(PCICR, PCIE1);
#elif OOK_PIN >= 8
  bitSet(PCMSK0, OOK_PIN - 8);
  bitSet(PCICR, PCIE0);
#else
  bitSet(PCMSK2, OOK_PIN);
  bitSet(PCICR, PCIE2);
#endif
}

void setup() {
  Serial.begin(57600);
  Serial.println("\n[ookScope]");
#ifdef OOK_POWER
  pinMode(OOK_POWER, OUTPUT);
  digitalWrite(OOK_POWER, HIGH);
#endif
  setupPinChangeInterrupt();
}

void loop() {
  if (next != fill) {
    cli();
    char b = buf[next++];
    sei();
    Serial.print(b);
  }
}
