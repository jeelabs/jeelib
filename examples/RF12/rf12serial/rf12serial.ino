// Serial data over RF12 demo, works in both directions
// 2009-04-17 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: rf12serial.pde 4727 2009-12-08 21:39:49Z jcw $

#include <Ports.h>
#include <RF12.h>
#include <RF12sio.h>

RF12 RF12;

void setup() {
    Serial.begin(9600);
    Serial.print("\n[rf12serial]");
    rf12_config();
}

void loop() {
    if (Serial.available())
        RF12.send(Serial.read());
    
    if (RF12.poll())
        Serial.print(RF12.read());
}
