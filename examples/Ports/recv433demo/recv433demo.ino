/// @dir recv433demo
/// Decoder for 433 MHz OOK pulses from remote control power switches, etc.
// 2009-02-27 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// The basic idea is to measure pulse widths between 0/1 and 1/0 transitions,
// and to keep track of pulse width sequences in a state machine. This code is
// set up to receive commands from the KlikAanKlikUit handheld remoe control
// transmitters, but the same mechanism can be used to receive a variety of
// signal types by adding extra state-machine recognizers.

#include <JeeLib.h>

enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

struct { uint8_t state; char bits; uint16_t data; } KAKU;

Port radio (3);

uint8_t KAKU_bit(uint8_t value) {
    KAKU.data = (KAKU.data << 1) | value;
    return ++KAKU.bits != 12 ? OK : DONE;
}

ISR(PCINT1_vect) {
    // width is the pulse length in usecs, for either polarity
    static uint16_t last;
    uint16_t width = micros() - last;
    last += width;

    // KAKU pulse widths are (S) 1x and (L) 3x 375 usec (0 = SLSL, 1 = SLLS)
    if (KAKU.state != DONE) {
        uint8_t w = (width + 360) / 540; // distinguish at 180/720/1260 usec
        switch (w) {
            case 0: // 0..179 usec
            case 3: // 1260..1799 usec
                KAKU.state = UNKNOWN; break;
            case 1: // 180..719 usec
            case 2: // 720..1259 usec
                switch (KAKU.state) {
                    case OK: // start of new data bit
                        KAKU.state = w == 1 ? T0 : UNKNOWN; break;
                    case T0: // short pulse seen
                        KAKU.state = w == 2 ? T1 : UNKNOWN; break;
                    case T1: // short + long pulse seen
                        KAKU.state += w; break;
                    case T2: // short + long + short pulse seen
                        KAKU.state = w == 2 ? KAKU_bit(0) : UNKNOWN; break;
                    case T3: // short + long + long pulse seen
                        KAKU.state = w == 1 ? KAKU_bit(1) : UNKNOWN; break;
                }
                break;
            default: // 1800..UP usec
                KAKU.state = OK; KAKU.bits = 0; KAKU.data = 0; break;
        }
    }
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[recv433demo]");
    
    radio.mode2(INPUT);
    
    // enable pin change interrupts on PC2
    PCMSK1 = bit(2);
    PCICR |= bit(PCIE1);
}

void loop() {
    if (KAKU.state == DONE) {
        Serial.print("KAKU ");
        Serial.println(KAKU.data, HEX);
        KAKU.state = UNKNOWN;
    }
}
