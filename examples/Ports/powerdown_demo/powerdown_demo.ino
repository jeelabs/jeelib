/// @dir powerdown_demo
/// Sample code to power down a JeeNode completely.
// 2010-08-16 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

void setup () {
    // turn the radio off completely
    rf12_initialize(17, RF12_868MHZ);
    rf12_sleep(RF12_SLEEP);

    // blink the blue LED three times, just because we can
    PORTB |= bit(1);
    DDRB |= bit(1);
    for (byte i = 0; i < 6; ++i) {
        delay(100);
        PINB = bit(1); // toggles
    }

    // stop responding to interrupts
    cli();
    
    // zzzzz... this code is now in the Ports library
    Sleepy::powerDown();
}

void loop () {}
