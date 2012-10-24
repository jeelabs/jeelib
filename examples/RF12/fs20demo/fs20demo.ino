/// @dir fs20demo
/// This example sends commands to the Conrad/ELV 868 MHz FS20 units via OOK.
// 2009-02-21 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// Note thar RFM12B radios are not really designed for OOK (on-off keying),
// but this can be simulated anyway by simply turning the transmitter on and 
// off via the SPI interface. Doing so takes about 25 usecs, so the delays
// used for encoding simple bit patterns need to be adjusted accordingly.

#include <JeeLib.h>
#include <util/parity.h>

void sendBits(uint16_t data, uint8_t bits) {
    if (bits == 8) {
        ++bits;
        data = (data << 1) | parity_even_bit(data);
    }
    for (uint16_t mask = bit(bits-1); mask != 0; mask >>= 1) {
        // Timing values empirically obtained, and used to adjust for on/off
        // delay in the RF12. The actual on-the-air bit timing we're after is
        // 600/600us for 1 and 400/400us for 0 - but to achieve that the RFM12B
        // needs to be turned on a bit longer and off a bit less. In addition
        // there is about 25 uS overhead in sending the on/off command over SPI.
        // With thanks to JGJ Veken for his help in getting these values right.
        int width = data & mask ? 600 : 400;
        rf12_onOff(1);
        delayMicroseconds(width + 150);
        rf12_onOff(0);
        delayMicroseconds(width - 200);
    }
}

void fs20cmd(uint16_t house, uint8_t addr, uint8_t cmd) {
	uint8_t sum = 6 + (house >> 8) + house + addr + cmd;
	for (uint8_t i = 0; i < 3; ++i) {
		sendBits(1, 13);
		sendBits(house >> 8, 8);
		sendBits(house, 8);
		sendBits(addr, 8);
		sendBits(cmd, 8);
		sendBits(sum, 8);
		sendBits(0, 1);
		delay(10);
	}
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[fs20demo]");
    
    rf12_initialize(0, RF12_868MHZ);
}

void loop() {  
    Serial.println("on");
	fs20cmd(0x1234, 1, 17);
	delay(2000);
	
    Serial.println("off");
	fs20cmd(0x1234, 1, 0);
	delay(5000);
}
