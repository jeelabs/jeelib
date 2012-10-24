/// @dir tsl230demo
/// Ports interface to the TSL230 light sensor, see SparkFun # 08940.
// 2009-02-21 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

// the TSL230 light sensor chip is connected as follows:
//      pin 1: S0   port 3 AIO
//      pin 2: S1   port 3 DIO
//      pin 3: OE   GND
//      pin 4: GND  GND
//      pin 5: VDD  +3.3V
//      pin 6: OUT  port 4 DIO
//      pin 7: S2   VDD
//      pin 8: S3   VDD

Port range (3), light (4);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[tsl230demo]");
    
    range.mode(OUTPUT);
    range.mode2(OUTPUT);
    light.mode(INPUT);
    
    // average sensitivity: 10x
    range.digiWrite(1);     // S1
    range.digiWrite2(0);    // S0
}

void loop() {
    delay(1000);
    
    Serial.print("\nTSL ");
    Serial.print(light.pulse(HIGH)); // measure high pulse width
}
