/// @dir bmp085demo
/// Ports demo, reads out a BMP085 sensor connected via I2C.
/// @see http://jeelabs.org/2010/06/30/going-for-gold-with-the-bmp085/
// 2009-02-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// 2010-05-22: added support for all resolution modes
// 2010-05-25: extended to also broadcast all readings over wireless
// 2010-06-17: add power saving logic, should reduce consumption by over 90%
// 2010-06-24: improved power savings, several "hot spots" optimized

// see http://jeelabs.org/2010/06/20/battery-savings-for-the-pressure-plug/
// see http://jeelabs.org/2010/06/30/going-for-gold-with-the-bmp085/

#include <JeeLib.h>
#include <PortsBMP085.h>
#include <avr/sleep.h>

PortI2C two (2);
BMP085 psensor (two, 3); // ultra high resolution
MilliTimer timer;

// This power-saving code was shamelessly stolen from the rooms.pde sketch,
// see http://code.jeelabs.org/viewvc/svn/jeelabs/trunk/jeemon/sketches/rooms/

EMPTY_INTERRUPT(WDT_vect); // just wakes us up to resume

static void watchdogInterrupts (char mode) {
    MCUSR &= ~(1<<WDRF); // only generate interrupts, no reset
    cli();
    WDTCSR |= (1<<WDCE) | (1<<WDE); // start timed sequence
    WDTCSR = mode >= 0 ? bit(WDIE) | mode : 0;
    sei();
}

static void lowPower (byte mode) {
    // prepare to go into power down mode
    set_sleep_mode(mode);
    // disable the ADC
    byte prrSave = PRR, adcsraSave = ADCSRA;
    ADCSRA &= ~ bit(ADEN);
    PRR |=  bit(PRADC);
    // zzzzz...
    sleep_mode();
    // re-enable the ADC
    PRR = prrSave;
    ADCSRA = adcsraSave;
}

static byte loseSomeTime (word msecs) {
    // only slow down for periods longer than the watchdog granularity
    if (msecs >= 16) {
        // watchdog needs to be running to regularly wake us from sleep mode
        watchdogInterrupts(0); // 16ms
        for (word ticks = msecs / 16; ticks > 0; --ticks) {
            lowPower(SLEEP_MODE_PWR_DOWN); // now completely power down
            // adjust the milli ticks, since we will have missed several
            extern volatile unsigned long timer0_millis;
            timer0_millis += 16;
        }
        watchdogInterrupts(-1); // off
        return 1;
    }
    return 0;
}

// End of new power-saving code.

void setup() {
    Serial.begin(57600);
    Serial.print("\n[bmp085demo]");
    rf12_initialize(3, RF12_868MHZ, 5); // 868 Mhz, net group 5, node 3
    
    psensor.getCalibData();
}

void loop() {
    // spend most of the waiting time in a low-power sleep mode
    // note: the node's sense of time is no longer 100% accurate after sleeping
    rf12_sleep(RF12_SLEEP);          // turn the radio off
    loseSomeTime(timer.remaining()); // go into a (controlled) comatose state
    
    while (!timer.poll(1000))
        lowPower(SLEEP_MODE_IDLE);   // still not running at full power

    // sensor readout takes some time, so go into power down while waiting
    // int32_t traw = psensor.measure(BMP085::TEMP);
    // int32_t praw = psensor.measure(BMP085::PRES);
    
    psensor.startMeas(BMP085::TEMP);
    loseSomeTime(16);
    int32_t traw = psensor.getResult(BMP085::TEMP);

    psensor.startMeas(BMP085::PRES);
    loseSomeTime(32);
    int32_t praw = psensor.getResult(BMP085::PRES);
    
    struct { int16_t temp; int32_t pres; } payload;
    psensor.calculate(payload.temp, payload.pres);
    
    // this code is not needed for use as remote node, keep it for debugging
    Serial.print("\nBMP ");
    Serial.print(traw);
    Serial.print(' ');
    Serial.print(praw);    
    Serial.print(' ');
    Serial.print(payload.temp);
    Serial.print(' ');
    Serial.print(payload.pres);
    Serial.flush();
    
    rf12_sleep(RF12_WAKEUP);         // turn radio back on at the last moment
    
    MilliTimer wait;                 // radio needs some time to power up, why?
    while (!wait.poll(5)) {
        rf12_recvDone();
        lowPower(SLEEP_MODE_IDLE);
    }
        
    rf12_sendNow(0, &payload, sizeof payload);
    rf12_sendWait(1); // sync mode!
}
