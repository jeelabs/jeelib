/// @dir blink_ports
/// Ports library demo, blinks leds on all 4 ports in slightly different ways.
/// @see http://jeelabs.org/2009/02/14/ports-library-for-arduino/
// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

Port one (1);
Port two (2);
Port three (3);
Port four (4);

// leds are connected to pins 2 (DIO) and 3 (GND) with a series resistor

void setup() {
	one.mode(OUTPUT);
	two.mode(OUTPUT);
	three.mode(OUTPUT);
	four.mode(OUTPUT);
}

void loop() {
	uint16_t ten = millis() / 10;
	
	one.digiWrite(ten & 0x10);  // led 1 blinks every 2x 0.16 sec
	four.digiWrite(ten & 0x80); // led 4 blinks every 2x 1.28 sec
	
	// ports 2 and 3 have support for PWM output
	// use bits 0..7 of ten as PWM output level
	// use 8 for up/down choice, i.e. make level ramp up and then down
	uint8_t level = ten;
	if (ten & 0x100)
		level = ~level;
		
	// leds 2 and 3 light up/down in opposite ways every 2x 2.56 sec
	two.anaWrite(level);
	three.anaWrite(~level);
}
