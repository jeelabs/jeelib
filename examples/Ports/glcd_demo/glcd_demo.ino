/// @dir glcd_demo
/// Demo display for the Graphics Boad.
/// @see http://jeelabs.org/2010/11/15/meet-the-graphics-board/
// 2010-11-14 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <ST7565.h>
#include <JeeLib.h>
#define BGLIGHT 3 // Backlight (IRQ) Pin

ST7565 glcd(14, 4, 17, 7);

void setup () {
    Serial.begin(57600);
    Serial.println("\n[glcd_demo]");
    rf12_initialize(1, RF12_868MHZ);
    rf12_sleep(RF12_SLEEP);
    
    glcd.st7565_init();
    glcd.st7565_command(CMD_DISPLAY_ON);
    glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
    glcd.st7565_set_brightness(0x15);
    glcd.clear();
    pinMode(BGLIGHT, OUTPUT); 
    for (int i = 1; i < 255; i++) {
        analogWrite(BGLIGHT, i); 
        delay(10);
    } // fade in

    // draw a string at a location
    glcd.drawstring(40, 0, "ARDUINO");
    glcd.drawstring(10, 2, "ST7565 128x64 GLCD");
    glcd.drawstring(22, 4, "Graphics Board");
    glcd.drawstring(20, 6, "JeeLabs.org/gb1");

    glcd.drawcircle(5, 5, 5, WHITE);
    glcd.fillcircle(121, 5, 5, WHITE);
    glcd.fillcircle(6, 58, 5, WHITE);
    glcd.drawcircle(121, 58, 5, WHITE);

    glcd.drawline(40, 9, 81, 9, WHITE);
    glcd.drawline(40, 11, 81, 11, WHITE);
    glcd.drawline(0, 42, 14, 28, WHITE);
    glcd.drawline(112, 42, 126, 28, WHITE);
    glcd.drawrect(0, 28, 127, 15, WHITE);

    glcd.display();
    for (int j = 255; j >= 1; j--) {
        analogWrite(BGLIGHT, j); 
        delay(50);
    } // fade out
    Sleepy::powerDown();
}

void loop () {}
