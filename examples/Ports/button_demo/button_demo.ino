// Show how the BlinkPlug's buttonCheck function works
// 2010-08-23 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: button_demo.pde 5961 2010-08-23 15:50:03Z jcw $

#include <Ports.h>
#include <RF12.h> // needed to avoid a linker error :(

BlinkPlug blink (1);
MilliTimer everySecond;

void setup () {
    Serial.begin(57600);
    Serial.println("\n[button_demo]");
}

void loop () {
    byte event = blink.buttonCheck();
    switch (event) {
        
    case BlinkPlug::ON1:
        Serial.println("  Button 1 pressed"); 
        break;
    
    case BlinkPlug::OFF1:
        Serial.println("  Button 1 released"); 
        break;
    
    case BlinkPlug::ON2:
        Serial.println("  Button 2 pressed"); 
        break;
    
    case BlinkPlug::OFF2:
        Serial.println("  Button 2 released"); 
        break;
    
    default:
        // report these other events only once a second
        if (everySecond.poll(1000)) {
            switch (event) {
                case BlinkPlug::SOME_ON:
                    Serial.println("SOME button is currently pressed");
                    break;
                case BlinkPlug::ALL_OFF:
                    Serial.println("NO buttons are currently pressed");
                    break;
            }
        }
    }
}
