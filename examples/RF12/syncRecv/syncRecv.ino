/// @dir syncRecv
/// Try to receive periodic transmissions with minimal power consumption.
/// @see http://jeelabs.org/2012/11/02/synchronised-reception/
// 2012-10-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/eeprom.h>

#define SEND_FREQ   RF12_868MHZ   // listening frequency
#define SEND_GROUP  5             // listening net group
#define SEND_ID     9             // listen for this node ID

#define CYCLE_TIME  3000          // expected cycle, milliseconds

#define EEADDR ((word*) 0)        // save estimate at this EEPROM address

word estimate;      // packet expected every 3s
word window;        // start with +/- 17 % window, power of 2
byte missed;        // count how often in a row receive failed
uint32_t lastRecv;  // time of last reception

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static void radioSleep (word ms) {
  rf12_sleep(RF12_SLEEP);
  Sleepy::loseSomeTime(ms);
  rf12_sleep(RF12_WAKEUP);
}

static bool isDesiredPacket () {
  return rf12_recvDone() && rf12_crc == 0 && rf12_hdr == SEND_ID;
}

static bool estimateWithWatchdog () {
  // first wait indefinitely for a packet to come in
  while (!isDesiredPacket())
    ;

  // sleep until estimate - window
  word lowEstimate = estimate - window;
  uint32_t start = millis();
  radioSleep(lowEstimate);

  // using the millis() timer, measure when the next packet comes
  do {
    lastRecv = millis();
    if (lastRecv - start > estimate + window)
      return false;
  } while (!isDesiredPacket());

  estimate = lastRecv - start;
  int offset = (estimate - lowEstimate) - window;

  Serial.print(" got "); Serial.print(estimate);
  Serial.print(", window = "); Serial.print(window);
  Serial.print(", offset = "); Serial.println(offset);
  return true;
}

static void chooseEstimate () {
  // use estimate in EEPROM, if it's sensible
  estimate = eeprom_read_word(EEADDR);
  if (estimate < CYCLE_TIME - CYCLE_TIME/5 ||
      estimate > CYCLE_TIME + CYCLE_TIME/5)
    estimate = CYCLE_TIME;
  Serial.print("start estimate = "); Serial.println(estimate);
  // narrow down estimate from 20 % -> 2 %
  window = estimate / 5; 
  while (window >= 16)
    if (estimateWithWatchdog())
      window /= 2;
  // save best estimaet so far
  eeprom_write_word(EEADDR, estimate);
  Serial.print("save estimate "); Serial.println(estimate);
}

void setup () {
  Serial.begin(57600);
  Serial.println("\n[syncRecv]");
  rf12_initialize(1, SEND_FREQ, SEND_GROUP); // we never send
  chooseEstimate();
}

void loop () {
  uint32_t now = millis();
  byte lost = (now - lastRecv + estimate/2) / estimate;
  Serial.print(" #"); Serial.print(lost);
  if (lost > 20)
    chooseEstimate();
  else {
    uint32_t predict = lastRecv + (lost + 1) * estimate;
    word sleep = predict - now - window;

    Serial.print(" s "); Serial.print(sleep);
    Serial.flush(); delayMicroseconds(250);

    radioSleep(sleep);

    uint32_t recvTime;
    do {
      recvTime = millis();
      if (recvTime > predict + window)
        return;
    } while (!isDesiredPacket());

    estimate = (recvTime - lastRecv) / (lost + 1);
    int offset = (estimate - sleep) - window;
    lastRecv = recvTime;

    Serial.print(" e "); Serial.print(estimate);
    Serial.print(" w "); Serial.print(window);
    Serial.print(" o "); Serial.print(offset);
    Serial.print(" rf "); Serial.println(offset + window);
  }
}
