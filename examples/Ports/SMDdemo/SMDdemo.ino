/// @dir SMDdemo
/// Test sketch for the SMD Kit - continuously toggle all the I/O lines.
// 2010-03-18 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

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
