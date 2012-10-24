/// @dir kaku_demo
/// This example sends commands to the KlikAanKlikUit units via OOK at 433 Mhz.
// 2009-02-21 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// Note that 868 MHz RFM12B's can send 433 MHz just fine, even though the RF
// circuitry is presumably not optimized for that band. Maybe the range will
// be limited, or maybe it's just because 868 is nearly a multiple of 433 ?

#include <JeeLib.h>
#include <util/parity.h>

// Turn transmitter on or off, but also apply asymmetric correction and account
// for 25 us SPI overhead to end up with the proper on-the-air pulse widths.
// With thanks to JGJ Veken for his help in getting these values right.
static void ookPulse(int on, int off) {
    rf12_onOff(1);
    delayMicroseconds(on + 150);
    rf12_onOff(0);
    delayMicroseconds(off - 200);
}

static void kakuSend(char addr, byte device, byte on) {
    int cmd = 0x600 | ((device - 1) << 4) | ((addr - 1) & 0xF);
    if (on)
        cmd |= 0x800;
    for (byte i = 0; i < 4; ++i) {
        for (byte bit = 0; bit < 12; ++bit) {
            ookPulse(375, 1125);
            int on = bitRead(cmd, bit) ? 1125 : 375;
            ookPulse(on, 1500 - on);
        }
		ookPulse(375, 375);
		delay(11); // approximate
    }
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[kaku_demo]");
    
    rf12_initialize(0, RF12_433MHZ);
}

void loop() {  
    Serial.println("off");
    kakuSend('B', 1, 0);
    delay(2000);
    
    Serial.println("on");
    kakuSend('B', 1, 1);
    delay(5000);
}
