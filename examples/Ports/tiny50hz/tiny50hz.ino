// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/

#include <JeeLib.h>
#include <PortsLCD.h>

PortI2C myI2C (1);
LiquidCrystalI2C lcd (myI2C);

#define ORDER 31
int values[ORDER];
int vmin, vmax;

static void led (byte on) {
#define LED 1
    bitSet(DDRB, LED);
    bitWrite(PORTB, LED, on);
}

static void setupAdc () {
    // 2.56V int ref, C2/B4 + C3/B3 diff, 20x
    //ADMUX = bit(REFS2) | bit(REFS1) | bit(MUX2) | bit(MUX1) | bit(MUX0);
    // 1.1V int ref, C2/B4 + C3/B3 diff, 20x
    ADMUX = bit(REFS1) | bit(MUX2) | bit(MUX1) | bit(MUX0);
    // disable these pins for digital I/O
    DIDR0 |= bit(ADC2D) | bit(ADC3D);
    // bipolar
    ADCSRB = bit(BIN);
    // clock/64 i.e. 128 KHz
    ADCSRA = bit(ADEN) | bit(ADPS2) | bit(ADPS1);
}

static int readAdc () {
  ADCSRA |= bit(ADSC);
  while (bitRead(ADCSRA, ADSC))
    ;
  return ADC;
}

static void measure () {
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
      accum += last - values[next];
      if (++next >= ORDER)
        next = 0;
      if (accum < vmin) vmin = accum;
      if (accum > vmax) vmax = accum;
    }
}

void setup () {
    led(0);
    lcd.begin(16, 2);
    lcd.print("Hello, world!");
    setupAdc();    
}

void loop () {
    led(1);
    delay(100);
    led(0);
    delay(900);

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
}
