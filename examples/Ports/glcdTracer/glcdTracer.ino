/// @dir glcdTracer
/// Very simple "Logic Analyzer", using the Graphics Board as display device.
/// @see http://jeelabs.org/2010/11/25/2-channel-logic-analyzer/
// 2010-11-20 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include "ST7565.h"

ST7565 glcd(14, 4, 17, 7);
byte samples[128];

static byte get2bits() {
    // bit 1 = AIO2, bit 0 = DIO2
    return (PINC & bit(1)) | bitRead(PIND, 5);
}

static void sample() {
    memset(samples, 0, sizeof samples);
    
    // wait for a transition on either input
    byte v = get2bits();
    while (get2bits() == v)
        ;
    
    // take 512 2-bit samples using a 128-byte buffer
    // first 128 samples in bits 0..1, 2nd in 2..3, etc.
    for (byte i = 0; i < 8; i += 2)
        for (byte j = 0; j < 128; ++j) {
            samples[j] |= get2bits() << i;
            delayMicroseconds(50); // approx 10 KHz
        }
}

void setup () {
    Serial.begin(57600);
    Serial.println("\n[glcd_demo]");
    
    glcd.st7565_init();
    glcd.st7565_command(CMD_DISPLAY_ON);
    glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
    glcd.st7565_set_brightness(0x15);
    
    digitalWrite(5, 1);     // DIO2 pull-up
    digitalWrite(15, 1);    // AIO2 pull-up
}

void loop () {
    glcd.clear();
    
    // sample quickly with interrupts disabled
    cli();
    sample();
    sei();

    for (byte i = 0; i < 4; ++i) {
        for (byte j = 0; j < 128; ++j) {
            if (bitRead(samples[j], 2*i))
                glcd.drawline(j, 16*i+8, j, 16*i+4, 1);
            else
                glcd.setpixel(j, 16*i+7, 1);
            if (bitRead(samples[j], 2*i+1))
                glcd.drawline(j, 16*i+14, j, 16*i+10, 1);
            else
                glcd.setpixel(j, 16*i+13, 1);
        }
    }

    glcd.display();
    delay(200);
}
