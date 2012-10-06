/// @dir sht11demo
/// Ports demo, reads out a SHT11 sensor connected via "something like I2C".
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
