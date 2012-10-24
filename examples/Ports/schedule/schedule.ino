/// @dir schedule
/// Demo sketch demonstrating the Scheduler class.
// 2010-10-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

enum { TASK1, TASK2, TASK_LIMIT };

static word schedBuf[TASK_LIMIT];
Scheduler scheduler (schedBuf, TASK_LIMIT);

BlinkPlug leds (3);
byte led1, led2;

// this has to be added since we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void setup () {
    Serial.begin(57600);
    Serial.println("\n[schedule]");
    
    // turn the radio off completely
    rf12_initialize(17, RF12_868MHZ);
    rf12_sleep(RF12_SLEEP);

    leds.ledOff(1+2);
    
    // start both tasks 1.5 seconds from now
    scheduler.timer(TASK1, 15);
    scheduler.timer(TASK2, 15);
}

void loop () {
    switch (scheduler.pollWaiting()) {
        // LED 1 blinks regularly, once a second
        case TASK1:
            led1 = !led1;
            if (led1) leds.ledOn(1); else leds.ledOff(1);
            scheduler.timer(TASK1, 5);
            break;
        // LED 2 blinks with short on pulses and slightly slower
        case TASK2:
            led2 = !led2;
            if (led2) leds.ledOn(2); else leds.ledOff(2);
            scheduler.timer(TASK2, 11 - 10 * led2);
            break;
    }
}
