/// @dir memory_demo
/// Demo of the MemoryPlug and MemoryStream classes.
// 2009-12-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

// Requires a JeeLabs Memory Plug on port 4 and a Blink Plug on port 3
//
// To use the demo:
//    Open the Serial Monitor in Arduino
//    Type some text into the serial monitor, then hit return
//      to save the text to the memory plug
//
// Press button 1 on the Blink Plug  to print out your text
// Press button 2 on the Blink Plug to erase the text, and reset the counter

BlinkPlug buttons (3);

PortI2C i2cBus (4);
MemoryPlug mem (i2cBus);
MemoryStream stream (mem);

void setup () {
    Serial.begin(57600);
    Serial.println("\n[memory_demo]");

    if (mem.isPresent()) {
        mem.save(100, 123, "abc", 3);
        byte buf[] = { 1, 2, 3, 4, 5 };
        delay(10);
        mem.load(100, 122, buf, 5);
        for (byte i = 0; i < sizeof buf; ++i) {
            Serial.print(buf[i], DEC);
            Serial.print(' ');
        }
        Serial.println("READY");
    }
}

void loop () {
    if (Serial.available())
        stream.put(Serial.read());
        
    byte pushed = buttons.pushed();
    if (pushed & 1) {
        long pos = stream.position(1);
        stream.flush();
        stream.reset();
        Serial.println();
        while (--pos >= 0 && !Serial.available())
            Serial.print((char) stream.get());
        Serial.println();
        stream.reset();
    }
    if (pushed & 2)
        stream.reset();
}
