// Test sketch for the JeeSMD plug - continuously toggle all the I/O lines.
// 2010-03-18 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: SMDdemo.pde 4880 2010-03-18 13:36:04Z jcw $

byte value;

void setup () {
    UCSR0B = 0; // disable the UART
    for (byte i = 0; i < 20; ++i)
        pinMode(i, OUTPUT);
}

void loop () {
    value = !value;    
    for (byte i = 0; i < 20; ++i)
        digitalWrite(i, value);
    delay(500);
}
