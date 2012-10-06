/// @dir blink_timers
/// Blink two LEDs independently.
/// @see http://jeelabs.org/2012/03/16/millitimer-example/
// 2012-03-09 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

BlinkPlug myPlug (1); // expects a Blink Plug on port 1
MilliTimer timer1, timer2;

void setup () {
  Serial.begin(57600);
  Serial.println("\n[blink_timers]");
}

static void toggleLed (byte which) {
  if (myPlug.ledState() & which)
    myPlug.ledOff(which);
  else
    myPlug.ledOn(which);  
}

void loop () {
  if (timer1.poll(250))
    toggleLed(1);
  if (timer2.poll(600))
    toggleLed(2);
}
