// Streaming layer demo, transfers different types of data in both directions
// 2009-05-07 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: rf12stream.pde 4727 2009-12-08 21:39:49Z jcw $

#include <Ports.h>
#include <RF12.h>
#include <RF12sio.h>

MilliTimer testTimer;
RF12 RF12;

void setup() {
    Serial.begin(9600);
    Serial.println("\n[rf12stream]");
    rf12_config();
}

void loop() {
    if (testTimer.poll(5000) && RF12.ready()) {
        static uint8_t seq;
        Serial.println("T");
        RF12 << 0x4321 << 'a' << seq << 123456789L << (float) 123.45;
        RF12.send(++seq);
    }

    if (RF12.poll()) {
        Serial.print("Got ");

        int a0 = RF12.read();
        Serial.print(a0);
        
        int a1, a2, a3;
        long a4;
        float a5;
        RF12 >> a1 >> a2 >> a3 >> a4 >> a5;
        
        Serial.print(' ');
        Serial.print(a1, HEX);
        Serial.print(' ');
        Serial.print(a2);
        Serial.print(' ');
        Serial.print(a3);
        Serial.print(' ');
        Serial.print(a4);
        Serial.print(' ');
        Serial.print((int) (100 * a5));
        Serial.println();
    }
}
