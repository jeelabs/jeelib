/// @dir syncRecv
/// Try to receive periodic transmissions with minimal power consumption.
/// @see http://jeelabs.org/2012/11/02/synchronised-reception/
/// @see http://jeelabs.org/2012/11/03/losing-sync-is-bad/
/// @see http://jeelabs.org/2012/11/04/predicting-the-next-packet-time/
/// @see http://jeelabs.org/2012/11/05/verifying-synchronisation-over-time/
// 2012-10-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <avr/eeprom.h>

#define DEBUG       0             // serial port debugging
#define DEBUG_LED   1             // turn LED on PB1 on while receiving

#define SEND_FREQ   RF12_868MHZ   // listening frequency
#define SEND_GROUP  5             // listening net group
#define SEND_ID     9             // listen for this node ID

#define CYCLE_TIME  3000          // expected cycle, milliseconds
#define MIN_WIN     16            // minimum window size, power of 2
#define MAX_WIN     1024          // maximum window size, power of 2

#define EEADDR ((word*) 0)        // save estimate at this EEPROM address

word estimate;      // packet expected every 3s
word window;        // start with +/- 17 % window, power of 2
byte missed;        // count how often in a row receive failed
uint32_t lastRecv;  // time of last reception

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static void radioSleep (word ms) {
#if DEBUG_LED
  bitSet(PORTB, 1); // LED off
#endif
  rf12_sleep(RF12_SLEEP);
  Sleepy::loseSomeTime(ms);
  rf12_sleep(RF12_WAKEUP);
#if DEBUG_LED
  bitClear(PORTB, 1); // LED on
#endif
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

#if DEBUG
  Serial.print(" t "); Serial.print(lastRecv / 1000);
  Serial.print(" e "); Serial.print(estimate);
  Serial.print(" w "); Serial.print(window);
  Serial.print(" o "); Serial.println(offset);
#endif
  return true;
}

static void chooseEstimate () {
  // use estimate in EEPROM, if it's sensible
  estimate = eeprom_read_word(EEADDR);
  if (estimate < CYCLE_TIME - CYCLE_TIME/5 ||
      estimate > CYCLE_TIME + CYCLE_TIME/5)
    estimate = CYCLE_TIME;
#if DEBUG
  Serial.print("start "); Serial.println(estimate);
#endif
  // narrow down estimate from 20 % -> 2 %
  window = estimate / 5; 
  while (window >= 2 * MIN_WIN)
    if (estimateWithWatchdog())
      window /= 2;
  // save best estimate so far
  eeprom_write_word(EEADDR, estimate);
#if DEBUG
  Serial.print("save "); Serial.println(estimate);
#endif
}

static bool optimalSleep () {
  uint32_t now = millis();
  // make a guess as to how many packets we lost
  byte lost = (now - lastRecv + estimate/2) / estimate;

  // if things are really bad, resync from scratch
  if (lost > 20) {
    chooseEstimate();
    return false;
  }

  // double the window every 4 additional packets lost
  if (lost > 0 && lost % 4 == 0 && window < MAX_WIN)
    window *= 2;

  // make a prediction for when the next packet should arrive
  uint32_t predict = lastRecv + (lost + 1) * estimate;
  word sleep = predict - now - (lost ? window : 0);
  radioSleep(sleep);

  // listen to the radio until one window past predicted time
  uint32_t recvTime;
  do {
    recvTime = millis();
    if (recvTime > predict + window)
      return false;
  } while (!isDesiredPacket());

  // adjust our estimate and prepare for more packets
  word newEst = (recvTime - lastRecv) / (lost + 1);
  estimate = (5 * estimate + newEst) / 6; // 6-fold smoothing
  int offset = (estimate - sleep) - window;
  lastRecv = recvTime;

#if DEBUG
  Serial.print(" n "); Serial.print(lost);
  Serial.print(" e "); Serial.print(estimate);
  Serial.print(" w "); Serial.print(window);
  Serial.print(" o "); Serial.print(offset);
  Serial.print(" rf "); Serial.println(offset + window);
  Serial.flush(); delayMicroseconds(250);
#endif

  // successful reception means we can narrow down the window
  if (window > MIN_WIN)
    window /= 2;
  return true;  
}

void setup () {
  Serial.begin(57600);
  Serial.println("\n[syncRecv] " __TIME__);
  rf12_initialize(1, SEND_FREQ, SEND_GROUP); // we never send
#if DEBUG_LED
  bitSet(DDRB, 1);
  bitClear(PORTB, 1); // LED on
#endif
  chooseEstimate();
  Serial.print("cycle "); Serial.println(estimate);
  Serial.flush(); delayMicroseconds(250);
  window = MIN_WIN;
}

void loop () {
  if (optimalSleep()) {
    uint32_t now = millis();
    Serial.print((now/100000) % 10);
    Serial.print((now/10000) % 10);
    Serial.print((now/1000) % 10);
    Serial.print('.');
    Serial.print((now/100) % 10);
    Serial.print(':');
    for (byte i = 0; i < rf12_len; ++i) {
      Serial.print(' ');
      Serial.print(rf12_data[i], HEX);
    }
    Serial.println();
    Serial.flush(); delayMicroseconds(250);
  }
}
