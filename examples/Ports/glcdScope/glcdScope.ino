// Very simple 100 KHz DSO scope, using the Graphics Board as display device.
// 2010-11-19 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: glcdScope.pde 6219 2010-11-19 02:05:04Z jcw $

#include "ST7565.h"

ST7565 glcd(14, 4, 17, 7);
byte samples[128];

static void sample() {
    for (byte i = 0; i < 128; ++i) {
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

    for (byte i = 0; i < 128; ++i)
        glcd.setpixel(i, 63 - samples[i] / 4, 1);

    glcd.display();
    delay(200);
}
