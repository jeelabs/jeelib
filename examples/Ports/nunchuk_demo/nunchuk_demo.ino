/// @dir nunchuk_demo
/// Ports demo, interface to Nintendo's "Nunchuk", which simply uses I2C inside.
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C four (4, 15 /*PortI2C::KHZ100*/);

DeviceI2C nunchuk (four, 0x52);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[nunchuk_demo]");

    nunchuk.send();
    nunchuk.write(0x40);
    nunchuk.write(0x00);
    nunchuk.stop();
}

static uint8_t decode(uint8_t last) {
    return (nunchuk.read(last) ^ 0x17) + 0x17;
}

void loop() {
    delay(1000);
    
    Serial.print("\nCHUK ");

    nunchuk.send();
    nunchuk.write(0x00);
    nunchuk.stop();
    
    delay(100);

    nunchuk.receive();
    uint8_t joyX = decode(0);
    uint8_t joyY = decode(0);
    uint16_t accX = decode(0) << 2; 
    uint16_t accY = decode(0) << 2;
    uint16_t accZ = decode(0) << 2;
    uint8_t flags = decode(1);
    
    int zBtn = flags & 0x01;
    int cBtn = (flags >> 1) & 0x01;
    accX |= (flags >> 2) & 0x03;
    accY |= (flags >> 4) & 0x03;
    accZ |= (flags >> 6) & 0x03;

    Serial.print (joyX, DEC);
    Serial.print (' ');
    Serial.print (joyY, DEC);
    Serial.print (' ');
    Serial.print (accX, DEC);
    Serial.print (' ');
    Serial.print (accY, DEC);
    Serial.print (' ');
    Serial.print (accZ, DEC);
    Serial.print (' ');
    Serial.print (zBtn, DEC);
    Serial.print (' ');
    Serial.print (cBtn, DEC);
}
