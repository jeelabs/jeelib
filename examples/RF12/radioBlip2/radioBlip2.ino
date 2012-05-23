// Send out a radio packet every minute, consuming as little power as possible
// 2012-05-09 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/sleep.h>

#define BLIP_NODE 22  // wireless node ID to use for sending blips
#define BLIP_GRP  5   // wireless net group to use for sending blips
#define BLIP_ID   1   // set this to a unique ID to disambiguate multiple nodes
#define SEND_MODE 3   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2

struct {
  long ping;  // 32-bit counter
  byte id;    // identity, should be different for each node
  byte vcc1;  // VCC before transmit, 1.0V = 0 .. 6.0V = 250
  byte vcc2;  // VCC after transmit, will be sent in next cycle
} payload;

volatile bool adcDone;

// for low-noise/-power ADC readouts, we'll use ADC completion interrupts
ISR(ADC_vect) { adcDone = true; }

// this must be defined since we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static byte vccRead (byte count =4) {
  set_sleep_mode(SLEEP_MODE_ADC);
  ADMUX = bit(REFS0) | 14; // use VCC as AREF and internal bandgap as input
  bitSet(ADCSRA, ADIE);
  while (count-- > 0) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
  }
  bitClear(ADCSRA, ADIE);  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1024U) / (ADC + 1) - 50;
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
  rf12_initialize(BLIP_NODE, RF12_868MHZ, BLIP_GRP);
  // see http://tools.jeelabs.org/rfm12b
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  rf12_sleep(RF12_SLEEP);

  payload.id = BLIP_ID;
}

static byte sendPayload () {
  ++payload.ping;

  rf12_sleep(RF12_WAKEUP);
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, &payload, sizeof payload);
  rf12_sendWait(SEND_MODE);
  rf12_sleep(RF12_SLEEP);
}

// This code tries to implement a good survival strategy: when power is low,
// don't transmit - when power is even lower, don't read out the VCC level.
//
// With a 100 µF cap, normal packet sends can cause VCC to drop by some 0.6V,
// hence the choices below: sending at >= 2.7V should be ok most of the time.

#define VCC_OK    85  // >= 2.7V - enough power for normal 1-minute sends
#define VCC_LOW   80  // >= 2.6V - sleep for 1 minute, then try again
#define VCC_DOZE  75  // >= 2.5V - sleep for 5 minutes, then try again
                      //  < 2.5V - sleep for 60 minutes, then try again
#define VCC_SLEEP_MINS(x) ((x) >= VCC_LOW ? 1 : (x) >= VCC_DOZE ? 5 : 60)

// Reasoning is that when we're about to try sending and find out that VCC
// is far too low, then let's just send anyway, as one final sign of life.

#define VCC_FINAL 70  // <= 2.4V - send anyway, might be our last swan song

void loop() {
  byte vcc = payload.vcc1 = vccRead();
  
  if (vcc <= VCC_FINAL) { // hopeless, maybe we can get one last packet out
    sendPayload();
    vcc = payload.vcc2 = 1; // don't even try reading VCC after this send
  }

  if (vcc >= VCC_OK) { // enough energy for normal operation
    sendPayload();
    vcc = payload.vcc2 = vccRead(); // measure and remember the VCC drop
  }

  byte minutes = VCC_SLEEP_MINS(vcc);
  while (minutes-- > 0)
    Sleepy::loseSomeTime(60000);
}
