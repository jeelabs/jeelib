// Send out a radio packet every minute, consuming as little power as possible
// 2012-05-09 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/sleep.h>

#define BLIP_ID 1  // set this to a unique ID to disambiguate multiple nodes

struct {
  long ping;  // 32-bit counter
  byte id;    // identity, should be different for each node
  byte vcc1;  // VCC before transmit, 1.0V = 0 .. 6.0V = 250
  byte vcc2;  // VCC after transmit, will be sent in next cycle
} payload;

volatile bool adcDone;

// this must be added since we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// for low-noise/-power ADC readouts, we'll use interrupts
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
  cli();
  CLKPR = bit(CLKPCE);
#if defined(__AVR_ATtiny84__)
  CLKPR = 0; // div 1, i.e. speed up to 8 MHz
#else
  CLKPR = 1; // div 2, i.e. slow down to 8 MHz
#endif
  sei();
  rf12_initialize(17, RF12_868MHZ, 5);
  // see http://tools.jeelabs.org/rfm12b
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  payload.id = BLIP_ID;
}

void loop() {
  payload.vcc1 = vccRead();

  ++payload.ping;

  while (!rf12_canSend())
    rf12_recvDone();
  
  rf12_sendStart(0, &payload, sizeof payload);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);

  payload.vcc2 = vccRead();
  
  rf12_sleep(RF12_SLEEP);
  Sleepy::loseSomeTime(60000);
  rf12_sleep(RF12_WAKEUP);
}
