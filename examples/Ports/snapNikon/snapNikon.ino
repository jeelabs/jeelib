/// @dir snapNikon
/// Send a remote command to take a snapshot on a Nikon camera, using infrared.
// 2010-06-10 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// IR LED connected between DIO1 and GND with a 33 ohm resistor in series
// using direct pin I/O for speed and PINx output for fast toggling
//
// the timing info was found at http://www.bigmike.it/ircontrol/

void setup () {
    digitalWrite(4, 0); // off
    pinMode(4, OUTPUT);
}

// 38.4 Khz is 26 us cycle, so toggle at 13 us and correct for overhead
static void send(word us) {
    // this won't overflow for pulses up to 3327 us
    byte count = us / 13;
    do {
        PIND = bit(4); // toggles the IR LED pin
        delayMicroseconds(12);
    } while (--count);
    bitClear(PORTD, 4); // make sure the LED is off at the end
}

static void snapshot() {
    for (byte i = 0; i < 2; ++i) {
        send(2000);
        delay(28);
        send(400);
        delayMicroseconds(1600);
        send(400);
        delayMicroseconds(3600);
        send(400);
        delay(63);
    }
}

void loop () {
    snapshot();
    delay(3000);
}
