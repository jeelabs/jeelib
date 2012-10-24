/// @dir relay_demo
/// Small demo for the Relay Plug, receives wireless packets and sets relays.
// 2010-07-05 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

Port relays (1);

void setup () {
    rf12_initialize(17, RF12_868MHZ, 5);
    
    relays.digiWrite(0);
    relays.mode(OUTPUT);
    relays.digiWrite2(0);
    relays.mode2(OUTPUT);
}

void loop () {
    if (rf12_recvDone() && rf12_crc == 0 && rf12_len == 2) {
        relays.digiWrite(rf12_data[0]);
        relays.digiWrite2(rf12_data[1]);
    }
}
