/// @dir ir_recv_irq
/// Infrared receiver using the InfraredPlug class (interrupt version).
// 2010-10-12 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

InfraredPlug ir (2);
byte value;

ISR(PCINT1_vect) {
    ir.poll();
}

void setup () {
    Serial.begin(57600);
    Serial.println("\n[ir_recv_irq]");

    // enable pin change interrupts on PC1
    PCMSK1 = bit(1);
    PCICR |= bit(PCIE1);
}

void loop () {
    if (Serial.available()) {
        char c = Serial.read();
        if ('0' <= c && c <= '9')
            value = 10 * value + c - '0';
        else {
            if (c == 's')
                ir.configure(value); // 20 ms end-of-data gap
            value = 0;
        }
    }
    
    byte count = ir.done();
    if (count > 0) {
        const byte* data = ir.buffer();
        Serial.print("IR ");
        switch (ir.decoder(count)) {
            case InfraredPlug::NEC:
                Serial.print("NEC");
                for (byte i = 0; i < 4; ++i) {
                    Serial.print(' ');
                    Serial.print(data[i], DEC);
                }
                break;
            case InfraredPlug::NEC_REP:
                Serial.print("NEC REPEAT");
                break;
            default:
                for (byte i = 0; i < count; ++i) {
                    byte nibble = (data[i/2] >> (i % 2) * 4) & 0x0F;
                    Serial.print(nibble, HEX);
                }
        }
        Serial.println();
    }
}
