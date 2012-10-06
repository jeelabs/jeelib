/// @dir eemem
/// Hooking up an 64 kbyte AT24C512B serial EEPROM via I2C.
/// @see http://jeelabs.org/2009/09/19/memory-plug-v1/
// 2009-07-03 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// see http://www.jeelabs.org/2009/07/06/external-memory/

// the AT24C512B eeprom is connected as follows (JeeNode v3 pinout):
//		pin 1, 2, 3, 7: not connected
//		pin 4: GND, port pin 3
//		pin 5: SDA, DIO, port pin 2
//		pin 6: SCL, AIO, port pin 5
//		pin 8: +3V, port pin 4

#include <JeeLib.h>

PortI2C myport (4730);
DeviceI2C eemem (myport, 0x50);

void setup() {
	Serial.begin(57600);
	Serial.println("\n[eemem]");
}

void loop() {
	delay(1000);
	
	// read address 0x1234
	eemem.send();
	eemem.write(0x12);
	eemem.write(0x34);	
	eemem.receive();
	byte value = eemem.read(1);
	eemem.stop();

	Serial.println((int) value);
	
	// write incremented value to address 0x01234
	eemem.send();
	eemem.write(0x12);
	eemem.write(0x34);
	eemem.write(++value);
	eemem.stop();
}
