/// @dir highside
/// Experimental code to control four high-side DC power switches.
// 2011-06-05 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define CHANNELS 1

MilliTimer collectTimer;
byte count;
word total[CHANNELS];

struct {
  word low, high, avg;
} payLoad[CHANNELS];

void setup () {
  Serial.begin(57600);
  Serial.print("\n[highside]");
  rf12_config();
  
  for (byte i = 0; i < CHANNELS; ++i) {
    digitalWrite(4 + i, 1);
    pinMode(4 + i, OUTPUT);
    payLoad[i].low = 10000;
  }
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0 && rf12_len == 1) {
    for (byte i = 0; i < CHANNELS; ++i)
      digitalWrite(4 + i, !bitRead(rf12_data[0], i));
    if (RF12_WANTS_ACK)
      rf12_sendStart(RF12_ACK_REPLY, 0, 0);
  }
  
  if (collectTimer.poll(100)) {
    ++count;
    for (byte i = 0; i < CHANNELS; ++i) {
      word value = 1023 - analogRead(i);
      total[i] += value;
      word volt = map(value, 0, 1023, 0, 3300);
      if (volt < payLoad[i].low)
        payLoad[i].low = volt;
      if (volt > payLoad[i].high)
        payLoad[i].high = volt;
      payLoad[i].avg = map(total[i] / count, 0, 1023, 0, 3300);
    }
  }
  
  if (count >= 50 && rf12_canSend()) {
    rf12_sendStart(0, payLoad, sizeof payLoad);
    count = 0;
    for (byte i = 0; i < CHANNELS; ++i) {
      payLoad[i].low = 10000;
      payLoad[i].high = 0;
      payLoad[i].avg = 0;
      total[i] = 0;
    }
  }
}
