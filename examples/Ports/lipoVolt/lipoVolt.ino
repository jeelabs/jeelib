/// @dir lipoVolt
/// Continuously read out the JeeNode USB voltage of the LiPo battery.
// 2011-11-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// $Id: $

void setup () {
    Serial.begin(57600);
    Serial.println("\n[lipoVolt]");
}

void loop () {
    int millivolt = map(analogRead(6), 0, 1023, 0, 6600);
    Serial.print("LIPO ");
    Serial.println(millivolt);    
    delay(1000);
}
