/// @dir bandgap
/// Try reading the bandgap reference voltage to measure current VCC voltage.
/// @see http://jeelabs.org/2012/05/12/improved-vcc-measurement/
// 2012-04-22 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/sleep.h>

volatile bool adcDone;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

ISR(ADC_vect) { adcDone = true; }

static byte vccRead (byte count =4) {
  set_sleep_mode(SLEEP_MODE_ADC);
  ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
  bitSet(ADCSRA, ADIE);
  while (count-- > 0) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
  }
  bitClear(ADCSRA, ADIE);  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
  rf12_initialize(17, RF12_868MHZ, 5);
}

void loop() {  
  byte x = vccRead();
  Sleepy::loseSomeTime(16);

  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &x, sizeof x);
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);

  Sleepy::loseSomeTime(1024);
}
