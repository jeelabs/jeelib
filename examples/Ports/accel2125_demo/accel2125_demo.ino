// Port interface to the Parallax Memsic 2125 Accelerometer #28017
// 2009-02-18 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: accel2125_demo.pde 5402 2010-04-30 19:24:52Z jcw $

#include <Ports.h>
#include <RF12.h> // needed to avoid a linker error :(

// the accelerometer has 2 pulse-modulated outputs tied to port 4

Port accel2125 (4);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[accel2125_demo]");
    
    accel2125.mode(INPUT);
    accel2125.mode2(INPUT);
}

void loop() {
    delay(1000);
    
    Serial.print("\nA2125 ");
    Serial.print(accel2125.pulse(HIGH));
    Serial.print(' ');
    Serial.print(accel2125.pulse2(HIGH));
}
