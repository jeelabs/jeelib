/// @dir opto_demo
/// Demo of the opto-coupler plug.
// 2010-09-26 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

Port optoIn (3), optoOut (4), testIn (2), testOut (1);
byte state; // counts on each cycle, bits 3..0 used as test signals

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void setup () {
    rf12_initialize(9, RF12_868MHZ, 5);
    
    // connect to opto-coupler plug as inputs with pull-ups enabled 
    optoIn.digiWrite(1);
    optoIn.mode(INPUT);
    optoIn.digiWrite2(1);
    optoIn.mode2(INPUT);
    // connect to opto-coupler plug as output 
    optoOut.mode(OUTPUT);
    optoOut.mode2(OUTPUT);
    // the testIn port will be used to generate test signals to optoIn
    testIn.mode(OUTPUT);
    testIn.mode2(OUTPUT);
    // the testOut port will be used to verify test signals from optoOut
    testOut.digiWrite(1);   // pull-up
    testOut.mode(INPUT);
    testOut.digiWrite2(1);  // pull-up
    testOut.mode2(INPUT);
}

void loop () {
    ++state;

    testIn.digiWrite(bitRead(state, 0));
    testIn.digiWrite2(bitRead(state, 1));
    optoOut.digiWrite(bitRead(state, 2));
    optoOut.digiWrite2(bitRead(state, 3));
    delay(2);

    byte payload[8];
    payload[0] = bitRead(state, 0);
    payload[1] = bitRead(state, 1);
    payload[2] = bitRead(state, 2);
    payload[3] = bitRead(state, 3);
    payload[4] = optoIn.digiRead();
    payload[5] = optoIn.digiRead2();
    payload[6] = testOut.digiRead();
    payload[7] = testOut.digiRead2();
    
    rf12_sendNow(0, payload, sizeof payload);
    rf12_sendWait(2);

    Sleepy::loseSomeTime(1000);
}
