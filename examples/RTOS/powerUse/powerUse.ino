/// @dir powerUse
/// Test sketch to determine ChibiOS's low-power sleep current consumption
/// @see http://jeelabs.org/2013/05/28/idling-in-low-power-mode/
// 2013-05-27 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <ChibiOS_AVR.h>
#include <JeeLib.h>

const bool LOWPOWER = true; // set to true to enable low-power sleeping

// must be defined in case we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static WORKING_AREA(waThread1, 50);

void Thread1 () {
  while (true)
    chThdSleepMilliseconds(1000);
}

void setup () {
  rf12_initialize(1, RF12_868MHZ);
  rf12_sleep(RF12_SLEEP);
  chBegin(mainThread);
}

void mainThread () {
  chThdCreateStatic(waThread1, sizeof (waThread1),
                    NORMALPRIO + 2, (tfunc_t) Thread1, 0);

  while (true)
    loop();
}

void loop () {
  if (LOWPOWER)
    Sleepy::loseSomeTime(16); // minimum watchdog granularity is 16 ms
  else
    delay(16);
}
