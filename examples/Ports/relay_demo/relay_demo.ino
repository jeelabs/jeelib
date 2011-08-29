// Small demo for the Relay Plug, receives wireless packets and sets relays
// 2010-07-05 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: relay_demo.pde 5908 2010-08-15 21:15:24Z jcw $

#include <Ports.h>
#include <RF12.h>

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
