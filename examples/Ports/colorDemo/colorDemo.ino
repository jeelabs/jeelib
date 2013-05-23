// Demo of the Color Plug, based on the ColorPlug class in the Ports library
// 2011-12-23 <l.herlaar@uu.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
    
PortI2C myBus (1);
ColorPlug sensor (myBus, 0x39); // Sensor address is 0x39

// gain = 3 (64x) and prescaler = 0 (1x) => most sensitive sensor setting,
//   for use in low light conditions
// gain = 0 (1x) and prescaler = 6 (64x) => least sensitive sensor setting,
//   for use in case of overflows
// gain = 0 (1x) and prescaler = 0 (1x) => default sensor setting,
//   vary according to need
byte gain = 0;      // 0..3 (multiplier => 3 most sensitive, 1/4/16/64x)
byte prescaler = 1; // 0..6 (divider => 0 most sensitive, 1/2/4/8/16/32/64x) 

void setup () {
    Serial.begin(57600);
    Serial.println("\n[colorDemo]");
    sensor.begin();
    sensor.setGain(gain, prescaler);
}

void loop () {    
    Serial.println();  

    // Get 16-bit values for R, G, B and Clear
    const word* rgbcValues = sensor.getData(); 
    Serial.print("Red: ");
    Serial.print(rgbcValues[0]);
    Serial.print(", Green: ");
    Serial.print(rgbcValues[1]);
    Serial.print(", Blue: ");
    Serial.print(rgbcValues[2]);
    Serial.print(", Clear: ");
    Serial.print(rgbcValues[3]);
    Serial.print(", Gain: ");
    Serial.print(gain, DEC);
    Serial.print(", Prescaler: ");
    Serial.println(prescaler, DEC);
  
    // Get chromaticity and correlared color temp (CCT)
    const word* chromacct = sensor.chromaCCT(); 
    // See http://en.wikipedia.org/wiki/Color_temperature for explanation
    if (chromacct[0] > 0 && chromacct[1] > 0) {
      Serial.print("Chromaticity x: ");
      Serial.print(chromacct[0] * 0.001, 2);
      Serial.print(", y: ");
      Serial.print(chromacct[1] * 0.001, 2);
      Serial.print(", Correlated color temp: ");
      Serial.println(chromacct[2]); // CCT of 0K means invalid
      // Note: although the CCT can be calculated for any chromaticity
      // coordinate, the result is meaningful only if the light source
      // is nearly white
    } else {
      Serial.println("Chroma overflow");
      // Try different gain/prescaler values if overflows occur
    }

    delay(1000);
}
