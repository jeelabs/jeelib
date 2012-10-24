/// @dir input_demo
/// Demo for the input plug - read 16 analog input channels once a second.
// 2010-04-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

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
