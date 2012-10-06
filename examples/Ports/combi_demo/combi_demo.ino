/// @dir  combi_demo
/// Ports demo, interfacing to lots of things at the same time.
/// @see http://jeelabs.org/2009/02/22/all-together-now/
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this code is the combination of all sht11demo, bmp085demo, pir_demo, and
// nunchuk_demo sources, plus some #define tricks to make it all work as one

// this is the original idea, but I couldn't figure out the proper include paths
// so it has been disabled, with the equivalent full source code added instead:

#ifdef NEVER

#define setup setup1
#define loop loop1
#include "/path/to/sht11demo.pde"
#undef setup setup1
#undef loop loop1

#define setup setup2
#define loop loop2
#include "/path/to/bmp085demo.pde"
#undef setup setup2
#undef loop loop2

#define setup setup3
#define loop loop3
#include "/path/to/pir_demo.pde"
#undef setup setup3
#undef loop loop3

#define setup setup4
#define loop loop4
#include "/path/to/nunchuk_demo.pde"
#undef setup setup4
#undef loop loop4

void setup() {
    Serial.begin(57600);
    Serial.print("\n[combi_demo]");
    setup1();
    setup2();
    setup3();
    setup4();
}

void loop() {
    loop1();
    loop2();
    loop3();
    loop4();
}

#endif

#define setup setup1
#define loop loop1

// Ports demo, reads out a SHT11 sensor connected via "something like I2C"
// 2009-02-16 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <PortsSHT11.h>

SHT11 hsensor (1);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[sht11demo]");
    
    // omit this call to avoid linking in the CRC calculation code
    hsensor.enableCRC();
}

void loop() {
    delay(800); // enable the SHT11 at no more than 20% duty cycle

    Serial.print("\nSHT ");

    uint8_t error = hsensor.measure(SHT11::HUMI);        
    Serial.print(hsensor.meas[SHT11::HUMI]);
    Serial.print(' ');
    
    error |= hsensor.measure(SHT11::TEMP);
    Serial.print(hsensor.meas[SHT11::TEMP]);
    Serial.print(' ');
    
    Serial.print(error, DEC);
    
    // omit following code to avoid linking in floating point code
    float h, t;
    hsensor.calculate(h, t);
    Serial.print(' ');
    Serial.print(h);
    Serial.print(' ');
    Serial.print(t);
    
    // omit following code to avoid linking in the log() math code
    float d = hsensor.dewpoint(h, t);
    Serial.print(' ');
    Serial.print(d);
}

#undef setup setup1
#undef loop loop1

#define setup setup2
#define loop loop2

// Ports demo, reads out a BMP085 sensor connected via I2C
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <PortsBMP085.h>

PortI2C two (2);

BMP085 psensor (two);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[bmp085demo]");
    
    // may omit following call if calculate() won't be called later on
    psensor.getCalibData();
}

void loop() {
    delay(1000);

    Serial.print("\nBMP ");
    
    uint16_t traw = psensor.measure(BMP085::TEMP);
    Serial.print(traw);
    
    uint16_t praw = psensor.measure(BMP085::PRES);
    Serial.print(' ');
    Serial.print(praw);
    
    // omit following code to avoid linking in some complex calculation code
    int16_t temp;
    int32_t pres;
    psensor.calculate(temp, pres);
    Serial.print(' ');
    Serial.print(temp);
    Serial.print(' ');
    Serial.print(pres);
}

#undef setup setup2
#undef loop loop2

#define setup setup3
#define loop loop3

// Ports demo, reads out a digital PIR sensor signal and an analog LDR voltage.
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

Port pir (3);   // PIR sensor is connected to DIO (pin 2) of port 3
                // to demo analog I/O, connect an LDR from AIO to GND

uint8_t state;  // tracks previous state of the PIR sensor signal

void setup() {
    Serial.begin(57600);
    Serial.print("\n[pir_demo]");
    
    pir.mode(INPUT);
    pir.mode2(INPUT);
    pir.digiWrite2(1); // pull-up
}

void loop() {
    if (pir.digiRead() != state) {
        state = pir.digiRead();
        uint16_t light = pir.anaRead();
        
        Serial.print("\nPIR ");
        Serial.print(state ? "on " : "off");
        Serial.print(" LDR ");
        Serial.print(light);
    }
}

#undef setup setup3
#undef loop loop3

#define setup setup4
#define loop loop4

// Ports demo, interface to Nintendo's "Nunchuk", which simply uses I2C inside
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

PortI2C four (4, PortI2C::KHZ100);

DeviceI2C nunchuk (four, 0x52);

void setup() {
    Serial.begin(57600);
    Serial.print("\n[nunchuk_demo]");

    nunchuk.send();
    nunchuk.write(0x40);
    nunchuk.write(0x00);
    nunchuk.stop();
}

void loop() {
    delay(1000);
    
    Serial.print("\nCHUK ");

    nunchuk.send();
    nunchuk.write(0x00);
    nunchuk.stop();
    
    delay(100);

    nunchuk.receive();
    uint8_t joyX = nunchuk.read(0);
    uint8_t joyY = nunchuk.read(0);
    uint16_t accX = nunchuk.read(0) << 2; 
    uint16_t accY = nunchuk.read(0) << 2;
    uint16_t accZ = nunchuk.read(0) << 2;
    uint8_t flags = nunchuk.read(1);
    
    int zBtn = (flags & 0x01) != 0;
    int cBtn = (flags & 0x02) != 0;
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

#undef setup setup4
#undef loop loop4

void setup() {
    Serial.begin(57600);
    Serial.print("\n[combi_demo]");
    setup1();
    setup2();
    setup3();
    setup4();
}

void loop() {
    loop1();
    loop2();
    loop3();
    loop4();
}
