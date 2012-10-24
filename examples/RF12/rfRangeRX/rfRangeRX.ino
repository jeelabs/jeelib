/// @dir rfRangeRX
/// Receive sketch used to report quality of reception.
// 2011-05-14 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// Based on a sample implementation by Steve Evans (@tankslappa).

#include <JeeLib.h>
#include <PortsLCD.h>

#define BUFFER_SIZE 64
#define DISPLAY_INTERVAL 500 // ms

PortI2C myI2C (3);
LiquidCrystalI2C lcd (myI2C);
MilliTimer displayTimer;

byte timeBuf [BUFFER_SIZE]; // index is time slot, value is last packet
byte seqBuf [BUFFER_SIZE];  // index is last packet, value is time slot
char history [11];
byte lastSeq;

static void lcd3dig (byte x, byte y, byte value, char fill =' ') {
  lcd.setCursor(x, y);
  lcd.print(value >= 100 ? (char) ('0' + value / 100) : fill);
  lcd.print(value >= 10 ? (char) ('0' + (value / 10) % 10) : fill);
  lcd.print(value % 10);
}

static void gotPacket () {
  byte tenths = millis() / 100;
  // remember for each time slot what the last received packet was
  timeBuf[tenths % BUFFER_SIZE] = lastSeq;
  // remember for the last BUFFER_SIZE packets when they arrived
  seqBuf[lastSeq % BUFFER_SIZE] = tenths;
}

static byte recvCount (byte period) {
  // tenths and diff are bytes, so they are automatically modulo 256
  byte tenths = millis() / 100;
  byte n = 0;
  for (byte i = 0; i < sizeof seqBuf; ++i) {
    byte diff = tenths - seqBuf[i];
    if (diff <= period)
      ++n;
  }
  return n;
}

static void updateHistory () {
  for (byte i = 1; i < 10; ++i)
    history[10-i] = history[9-i];
  history[0] = '0';
  // tenths and diff are bytes, so they are automatically modulo 256
  byte tenths = millis() / 100;
  for (byte i = 0; i < DISPLAY_INTERVAL / 100; ++i) {
    byte pos = (tenths - i - 1) % BUFFER_SIZE;
    byte diff = lastSeq - timeBuf[pos];
    if (diff < 5)
      ++history[0];
  }
}

void setup () {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("xxx %/5s xxx %/s");
  lcd.setCursor(0, 1);
  lcd.print("#xxx >xxxxxxxxxx");
  rf12_initialize('R', RF12_868MHZ, 88);
  // synchronize the display to 0.1s clock transitions
  displayTimer.set(DISPLAY_INTERVAL - millis() % DISPLAY_INTERVAL - 1);
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0 && rf12_len == 1) {
    lastSeq = rf12_data[0];
    lcd3dig(1, 1, lastSeq, '0');
    gotPacket();
  }
    
  if (displayTimer.poll(DISPLAY_INTERVAL)) {
    // number of packets received in the last 5 seconds, as percentage
    lcd3dig(0, 0, recvCount(50) * 2);
    // number of packets received in the last second, as percentage
    lcd3dig(9, 0, recvCount(10) * 10);
    // show number of packets received in the last 10 display intervals
    updateHistory();
    lcd.setCursor(6, 1);
    lcd.print(history);
  }
}
