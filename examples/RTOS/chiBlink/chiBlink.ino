/// @dir chiBlink
/// Simple LED blinker using the ChibiOS real-time operating system
/// @see http://jeelabs.org/2013/05/24/blinking-in-real-time/
// 2013-05-23 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <ChibiOS_AVR.h>

static WORKING_AREA(waThread1, 50);

void Thread1 () {
  const uint8_t LED_PIN = 9;
  pinMode(LED_PIN, OUTPUT);
  
  while (true) {
    digitalWrite(LED_PIN, LOW);
    chThdSleepMilliseconds(100);
    digitalWrite(LED_PIN, HIGH);
    chThdSleepMilliseconds(500);
  }
}

void setup () {
  chBegin(mainThread);
}

void mainThread () {
  chThdCreateStatic(waThread1, sizeof (waThread1),
                    NORMALPRIO + 2, (tfunc_t) Thread1, 0);
  while (true)
    loop();
}

void loop () {
  delay(1000);
}
