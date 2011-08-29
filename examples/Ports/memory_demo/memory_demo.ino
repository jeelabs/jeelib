// Demo of the MemoryPlug and MemoryStream classes
// 2009-12-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

// pressing button 1 replays the collected data
// pressing button 2 clears the data
BlinkPlug buttons (3);

PortI2C i2cBus (4);
MemoryPlug mem (i2cBus);
MemoryStream stream (mem);

void setup () {
    Serial.begin(57600);
    Serial.println("\n[memory_demo]");

    if (mem.isPresent()) {
        mem.save(100, "abc", 123, 3);
        byte buf[] = { 1, 2, 3, 4, 5 };
        delay(10);
        mem.load(100, buf, 122, 5);
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
            Serial.print(stream.get());
        Serial.println();
        stream.reset();
    }
    if (pushed & 2)
        stream.reset();
}
