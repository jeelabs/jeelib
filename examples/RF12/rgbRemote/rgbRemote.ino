/// @dir rgbRemote
/// Control some LED strips, using settings received by wireless.
// 2010-10-01 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// see http://jeelabs.org/2010/06/15/remote-rgb-strip-control/
// and http://jeelabs.org/2010/10/03/software-pwm-at-1-khz/
//
// 2010-06-12: delayed init of RF12 driver, to make LED startup a bit quicker
// 2010-09-08: adapted for 2x rgb with 0..100% intensity control
// 2010-10-01: back to 255 intensity levels, but with new TCT0-based trick
//
// example settings:
//
//  off all     255,255,255,0,255,255,255,0,30s
//  min all     255,255,255,1,255,255,255,1,30s
//  max all     255,255,255,255,255,255,255,255,30s
//  max cold    255,115,38,255,255,115,38,255,30s
//  max medium  255,100,13,255,255,100,13,255,30s
//
//  calm warm   255,90,8,25,255,90,8,25,30s
//  max warm    255,90,8,255,255,90,8,255,30s
//
// TODO: save changes to EEPROM using interrupts, i.e. in the background
// TODO: staggered turn-on, to reduce the inrush current of time slot zero

#include <JeeLib.h>
#include <EEPROM.h>

#define DEBUG 0

#define BAND RF12_868MHZ // wireless frequency band
#define GROUP 5     // wireless net group
#define NODEID 30   // node id on wireless to which this sketch responds
#define NVALUES 8   // can be adjusted from 1..8 (must also adjust "masks")

static byte settings[NVALUES];  // 0 = red, 1 = green, 2 = blue, 3 = white
static byte slots[256];         // time slots, toggles whenever a bit is set

MilliTimer timer;

// which setting affects which I/O pin:
//  bits 0..3 = AIO1 .. AIO4
//  bits 4..7 = DIO1 .. DIO4
static const byte masks[NVALUES] = {
    0x20, 0x02, 0x80, 0x00, 0x08, 0x10, 0x01, 0x00
};

static void setupIO() {
    for (byte i = 0; i < NVALUES; ++i) {
        PORTC &= ~ (masks[i] & 0x0F);   // turn AIO pin off
        DDRC |= masks[i] & 0x0F;        // make pin an output
        PORTD &= ~ (masks[i] & 0xF0);   // turn DIO pin off
        DDRD |= masks[i] & 0xF0;        // make pin an output
    }    
}

static void prepareSlots() {
#if DEBUG
    Serial.print("RGB values:");
    for (byte i = 0; i < NVALUES; ++i) {
        Serial.print(' ');
        Serial.print(settings[i], DEC);
    }
    Serial.println();
#endif    

    // use 4th value as intensity control over RGB values
    settings[0] = (settings[0] * settings[3] + 127) / 255; 
    settings[1] = (settings[1] * settings[3] + 127) / 255; 
    settings[2] = (settings[2] * settings[3] + 127) / 255; 
    // also for the second RGB group
    settings[4] = (settings[4] * settings[7] + 127) / 255; 
    settings[5] = (settings[5] * settings[7] + 127) / 255; 
    settings[6] = (settings[6] * settings[7] + 127) / 255; 

    // fill the slots arrray with on-bits, as implied by the intensity values
    memset(slots, 0, sizeof slots);
    for (byte i = 0; i < NVALUES; ++i) {
        // get the requested PWM level
        word intensity = settings[i];
        // don't use intensities 1 and 254, as they would toggle within 4 us
        // convert 0..255 values to 0 = off, 2..254 = dimmed, 256 = on
        if (intensity > 0) ++intensity;     // change 1..255 to 2..256
        if (intensity == 255) ++intensity;  // change (original) 254 to 256
        // fill in the 1's
        byte mask = masks[i]; // map setting to corresponding I/O pin
        for (word i = 0; i < intensity; ++i)
            slots[i] |= mask;
    }
}

static void loadSettings() {
    for (byte i = 0; i < NVALUES; ++i)
        settings[i] = EEPROM.read(i);
    prepareSlots();
}

static void saveSettings() {
    for (byte i = 0; i < NVALUES; ++i)
        EEPROM.write(i, settings[i]);
    prepareSlots();
}

static void showSettings() {
}

void setup () {
#if DEBUG
    Serial.begin(57600);
    Serial.println("\n[rgbRemote]");
#endif
    setupIO();
    loadSettings();
    rf12_initialize(NODEID, BAND, GROUP);
}

void loop() {
    if (timer.poll(100) && rf12_recvDone()
                        && rf12_hdr == (RF12_HDR_DST | NODEID)
                        && rf12_crc == 0 && rf12_len == NVALUES) {
        // turn LEDs off before making changes (saving takes time!)
        PORTC &= 0xF0;
        PORTD &= 0x0F;
        memcpy(settings, (void*) rf12_data, rf12_len);
        saveSettings();
    }
    byte bits = slots[TCNT0];
    PORTC = (PORTC & 0xF0) | (bits & 0x0F);
    PORTD = (PORTD & 0x0F) | (bits & 0xF0);
}
