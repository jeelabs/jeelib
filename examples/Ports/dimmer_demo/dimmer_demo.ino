/// @dir dimmer_demo
/// Demo for the Dimmer plug.
/// @see http://jeelabs.org/2011/06/11/controlling-the-dimmer-plug/
// 2010-03-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C myBus (1);
DimmerPlug dimmer (myBus, 0x40);

int level = 0x1FFF;

void setup () {
    dimmer.begin();
    // set each channel to max brightness
    dimmer.setMulti(dimmer.PWM0, 255, 255, 255, 255,
                                 255, 255, 255, 255,
                                 255, 255, 255, 255,
                                 255, 255, 255, 255, -1);
    // set up for group blinking
    dimmer.setReg(dimmer.MODE2, 0x34);
    // blink rate: 0 = very fast, 255 = 10s
    dimmer.setReg(dimmer.GRPFREQ, 50);
    // blink duty cycle: 0 = full on, 255 = full off
    dimmer.setReg(dimmer.GRPPWM, 240);
    // let the chip do its thing for a while
    delay(10000);
    // set up for group dimming
    dimmer.setReg(dimmer.MODE2, 0x14);
    // gradually decrease brightness to minimum
    for (byte i = 100; i < 255; ++i) {
        dimmer.setReg(dimmer.GRPPWM, i);
        delay(100);
    }
    // the rest of the code dims individual channels 
    delay(2000);
}

void loop () {
    byte brightness = ++level;
    if (level & 0x100)
        brightness = ~ brightness;
    byte r = level & 0x0200 ? brightness : 0;
    byte g = level & 0x0400 ? brightness : 0;
    byte b = level & 0x0800 ? brightness : 0;
    byte w = level & 0x1000 ? brightness : 0;
    // set all 16 registers in one sweep
    dimmer.setMulti(dimmer.PWM0, w, b, g, r,
                                 w, b, g, r,
                                 w, b, g, r,
                                 w, b, g, r, -1);
}
