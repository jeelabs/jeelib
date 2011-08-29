// Very simple Spectrum Analyzer, using the Graphics Board as display device.
// 2010-11-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <ST7565.h>

extern int fix_fft(char[], char[], int, int);

ST7565 glcd(14, 4, 17, 7);
char samples[256], im[256];

static void sample() {
    for (word i = 0; i < 256; ++i) {
        loop_until_bit_is_set(ADCSRA, ADIF);
        bitSet(ADCSRA, ADIF);
        samples[i] = ADCH;
    }
}

void setup () {
    Serial.begin(57600);
    Serial.println("\n[glcd_demo]");
    
    glcd.st7565_init();
    glcd.st7565_command(CMD_DISPLAY_ON);
    glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
    glcd.st7565_set_brightness(0x15);

#if 1    
    pinMode(3, OUTPUT);
    digitalWrite(3, 1); // turn the backlight on
#endif

    analogRead(2); // run once to set up the ADC
    
    bitSet(ADMUX, ADLAR);   // left adjust result

    // the ADC clock will determine the sample rate
    ADCSRA = (ADCSRA & ~7) | 3; // values 2..7 are ok
    
    ADCSRB &= ~7;           // free-running
    bitSet(ADCSRA, ADATE);  // auto trigger
    bitSet(ADCSRA, ADSC);   // start conversion
}

void loop () {
    glcd.clear();
    
    // sample quickly with interrupts disabled
    cli();
    sample();
    sei();

    // perform a 2^8, i.e. 256-point Fast Fourier Transform
    memset(im, 0, sizeof im);
    fix_fft(samples, im, 8, 0);
    
    // display first half of the results, magnify values 3x
    for (byte i = 0; i < 128; ++i)
        glcd.drawline(i, 63 - samples[i] * 3, i, 63, 1);

    glcd.display();
}
