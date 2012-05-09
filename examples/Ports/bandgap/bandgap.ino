// Try reading the bandgap reference voltage to measure current VCC voltage.
// 2012-04-22 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/sleep.h>

volatile bool adcDone;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

ISR(ADC_vect) { adcDone = true; }

static int vccRead () {
  set_sleep_mode(SLEEP_MODE_ADC);
  ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
  bitSet(ADCSRA, ADIE);
  for (byte i = 0; i < 4; ++i) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
  }
  bitClear(ADCSRA, ADIE);  
  word x = ADC;
  return x ? (1100L * 1023) / x : -1;
}

void setup() {
  rf12_initialize(17, RF12_868MHZ, 5);
}

void loop() {  
  // convert millivolts to fit within a byte, i.e. 20 mV steps:
  //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  byte x = map(vccRead(), 1000, 6000, 0, 250);
  Sleepy::loseSomeTime(16);

  rf12_sleep(RF12_WAKEUP);
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, &x, sizeof x);
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);

  Sleepy::loseSomeTime(1024);
}
