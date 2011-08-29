// Demo for the input plug - read 16 analog input channels once a second
// 2010-04-19 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: input_demo.pde 5370 2010-04-27 01:27:03Z jcw $

#include <Ports.h>
#include <RF12.h> // needed to avoid a linker error :(

InputPlug input (1);

void setup () {
    Serial.begin(57600);
    Serial.println("\n[input_demo]");
    
    input.mode2(INPUT);
    input.digiWrite2(1); // pull-up, not a good idea in normal use
}

void loop () {
    Serial.print("INPUT");
    for (byte i = 0; i < 16; ++i) {
        input.select(i);
        Serial.print(' ');
        Serial.print(input.anaRead());
    }
    Serial.println();
        
    delay(1000);
}
