// Try reading the bandgap reference voltage to measure current VCC voltage.
// 2012-04-22 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

static int vccRead (byte us =250) {
  analogRead(6);    // set up "almost" the proper ADC readout
  bitSet(ADMUX, 3); // then fix it to switch to channel 14
  delayMicroseconds(us); // delay substantially improves accuracy
  bitSet(ADCSRA, ADSC);
	while (bit_is_set(ADCSRA, ADSC))
	  ;
  word x = ADC;
  return x ? (1100L * 1023) / x : -1;
}

void setup() {
  Serial.begin(57600);
  Serial.println("\n[bandgap]");
}

void loop() {  
  analogRead(0); // just to check that it doesn't affect result
  Serial.println(vccRead());
  delay(500);
}
