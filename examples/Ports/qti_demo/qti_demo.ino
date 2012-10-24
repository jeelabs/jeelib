/// @dir qti_demo
/// Port interface to the Parallax QTI sensor # 27401.
// 2009-02-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

// the QTI sensor is connected to port 1 as follows:
// B = pin 3 (GND), R = pin 4 (AIO), W = pin 5 (VCC)

Port qti (1);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[qti_demo]");
    
    qti.mode2(INPUT);
    qti.digiWrite2(1); // pull-up
}

void loop() {
    delay(1000);
    
    Serial.print("\nQTI ");
    Serial.print(qti.anaRead());
}
