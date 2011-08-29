// Demo of the BlinkPlug class
// 2009-12-09 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: blink_demo.pde 4730 2009-12-11 15:36:29Z jcw $

#include <Ports.h>
#include <RF12.h> // needed to avoid a linker error :(

BlinkPlug buttons (3);

void setup () {
    Serial.begin(57600);
    Serial.println("\n[blink_demo]");
    
    for (byte i = 0; i < 3; ++i) {
        buttons.ledOn(1);
        delay(500);
        buttons.ledOn(2);
        delay(500);
        buttons.ledOff(1+2);
        delay(500);
    }
}

int rate = 750;
MilliTimer blink;
byte on;

void loop () {
    byte pushed = buttons.pushed();
    if (pushed) {
        if ((pushed & 1) && rate < 15000)
            rate *= 2;
        if ((pushed & 2) && rate > 1)
            rate /= 2;
        blink.set(rate); // start with new rate right away
        Serial.println(rate);
    }
    if (buttons.state() == 1+2)
        rate = 750;
    if (blink.poll(rate)) {
        on = ! on;
        if (on)
            buttons.ledOn(1);
        else
            buttons.ledOff(1);
    }
}
