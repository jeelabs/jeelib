/// @dir blink_xmit
/// Ports library demo, this is the transmitter, see also blink_recv example.
/// @see http://jeelabs.org/2009/02/15/remote-ports/
// 2009-02-14 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

RemoteNode bob ('B', RF12_868MHZ);

RemotePort one (bob, 1);
RemotePort two (bob, 2);
RemotePort three (bob, 3);
RemotePort four (bob, 4);

void setup() {
    one.mode(OUTPUT);
    two.mode(OUTPUT);
    three.mode(OUTPUT);
    four.mode(OUTPUT);
}

void loop() {
    // exchange packets once every 80 msecs to keep up with the led blink rate
    // each packet + ack takes some 5 msecs, i.e. about 6% channel utilization
    // real-world uses will need to poll a bit slower, say once per second
    bob.poll(80);

    uint16_t ten = millis() / 10;
    
    one.digiWrite(ten & 0x10);  // led 1 blinks every 2x 0.16 sec
    four.digiWrite(ten & 0x80); // led 4 blinks every 2x 1.28 sec
    
    // ports 2 and 3 have support for PWM output
    // use bits 0..7 of ten as PWM output level
    // use 8 for up/down choice, i.e. make level ramp up and then down
    uint8_t level = ten;
    if (ten & 0x100)
        level = ~level;
        
    // leds 2 and 3 light up/down in opposite ways every 2x 2.56 sec
    two.anaWrite(level);
    three.anaWrite(~level);
}
