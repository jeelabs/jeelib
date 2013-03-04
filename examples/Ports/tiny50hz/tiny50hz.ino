/// @dir tiny50hz
/// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny84/85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/

#include <JeeLib.h>
#include <avr/sleep.h>

// for an ATtiny85, this compiles to a test code with an LCD + LED attached
// else we assume ATtiny84 and compile the real thing, including RF12 driver

#ifdef __AVR_ATtiny85__
#define TEST_CODE 1
#endif

#if TEST_CODE
#include <PortsLCD.h>
#define LED 1
PortI2C myI2C (1);
LiquidCrystalI2C lcd (myI2C);
#endif

#define ORDER 31
int values[ORDER];
int vmin, vmax;
struct { word last, prev; } payload;
byte lastTime;

static void led (byte on) {
#ifdef LED
  bitSet(DDRB, LED);
  bitWrite(PORTB, LED, on);
#endif
}

static void setupAdc () {
#if TEST_CODE
  // 1.1V int ref, C2/B4 + C3/B3 diff, 20x
  ADMUX = bit(REFS1) | bit(MUX2) | bit(MUX1) | bit(MUX0);
  // disable these pins for digital I/O
  DIDR0 |= bit(ADC2D) | bit(ADC3D);
  // bipolar
  ADCSRB = bit(BIN);
  // clock/8 i.e. 128 KHz 
  ADCSRA = bit(ADEN) | bit(ADPS1) | bit(ADPS0);
#else
  // 1.1V int ref, PA1(+) and PA0(-) diff, 20x
  ADMUX = bit(REFS1) | bit(MUX5) | bit(MUX3) | bit(MUX0);
  // disable these pins for digital I/O
  DIDR0 |= bit(ADC1D) | bit(ADC0D);
  // bipolar
  ADCSRB = bit(BIN);
  // clock/8 i.e. 128 KHz 
  ADCSRA = bit(ADEN) | bit(ADPS1) | bit(ADPS0);
#endif
}

static int readAdc () {
  ADCSRA |= bit(ADSC);
  while (bitRead(ADCSRA, ADSC))
    ;
  return ADC;
}

static void measure () {
  bitClear(PRR, PRADC); // power up the ADC
  ADCSRA |= bit(ADEN); // enable the ADC
  // initialize
  readAdc(); // discard first reading
  int accum = 0;
  for (byte i = 0; i < ORDER; ++i) {
    values[i] = readAdc();
    accum += values[i];
  }
  // acquire and determine min/max
  vmin = 32767;
  vmax = -32768;
  byte next = 0;
  for (word i = 0; i < 500; ++i) {
    int last = values[next];
    values[next] = readAdc();
    accum += values[next] - last;
    if (++next >= ORDER)
      next = 0;
    if (accum < vmin) vmin = accum;
    if (accum > vmax) vmax = accum;
  }
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
}

static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = bit(CLKPCE);
    CLKPR = mode;
    sei();
}

void setup () {
  setPrescaler(3); // div 8, i.e. 1 MHz
#if TEST_CODE
  lcd.begin(16, 2);
  lcd.print("Hello, world!");
#else
  rf12_initialize(17, RF12_868MHZ, 5);
  rf12_sleep(RF12_SLEEP);
#endif
  setupAdc();    
  led(0);
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  PRR = bit(PRTIM1) | bit(PRUSI) | bit(PRADC); // only keep timer 0 going
}

void loop () {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();
  byte nextTime = millis() >> 14; // 16384 ms
  if (nextTime == lastTime)
    return;
  lastTime = nextTime;
    
#if TEST_CODE
  led(1);
  delay(100);
  led(0);

  byte t = millis();
  measure();    
  t = (byte) millis() - t;

  lcd.setCursor(0, 1);
  lcd.print(vmax - vmin);
  lcd.print(' ');
  lcd.print(vmin);
  lcd.print('-');
  lcd.print(vmax);
  lcd.print(' ');
  lcd.print((int) t);
  lcd.print('.');
#else
  // if (millis() < 900000) return; // don't report in first 15m after power up
  measure();
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  payload.prev = payload.last;
  payload.last = vmax - vmin;
  bitClear(PRR, PRUSI); // enable USI h/w
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &payload, sizeof payload);
  rf12_sendWait(1);
  rf12_sleep(RF12_SLEEP);
  bitSet(PRR, PRUSI); // disable USI h/w
  setPrescaler(3); // div 8, i.e. 1 MHz
#endif
}
