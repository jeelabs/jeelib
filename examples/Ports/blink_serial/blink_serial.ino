/// @dir blink_serial
/// Controlling and reading out a Blink Plug over the serial port.
// 2012-12-21 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

BlinkPlug buttons (1);

void setup () {
  Serial.begin(57600);
  // Serial.println("\n[blink_serial]");
}

void loop () {
  // listen to incoming commands
  if (Serial.available())
    switch (Serial.read()) {
      case 'A': buttons.ledOn(1); break;
      case 'a': buttons.ledOff(1); break;
      case 'B': buttons.ledOn(2); break;
      case 'b': buttons.ledOff(2); break;
    }
  // look for button presses
  switch (buttons.buttonCheck()) {
    case BlinkPlug::ON1:  Serial.println("+1"); break;
    case BlinkPlug::OFF1: Serial.println("-1"); break;
    case BlinkPlug::ON2:  Serial.println("+2"); break;
    case BlinkPlug::OFF2: Serial.println("-2"); break;
  }
}
