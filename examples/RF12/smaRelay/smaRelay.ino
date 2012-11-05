/// @dir smaRelay
/// Read out SMA solar inverter via Bluetooth and relay readings using RFM12B.
// 2012-11-04 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define LED 9 // inverted logic

// this is the main driver, it calls the getByte() and) emitFinal() functions
#include "smaComms.h"

struct { word yield, total, acw, dcv[2], dcw[2]; } payload;

byte myAddrBT[] = { 0xCA,0xC2,0x46,0x66,0x06,0x00 }; // @TODO obtain from rn42
MilliTimer sendTimer;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static byte getByte () {
#if LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);
#endif
  while (!Serial.available())
    ;
#if LED
  digitalWrite(LED, 1);
#endif
  return Serial.read();
}

static void emitFinal () {
  // fill in packet length and prefix check
  packetBuf[1] = fill - packetBuf;
  packetBuf[3] = packetBuf[0] ^ packetBuf[1];
  // send the packet out
  for (byte i = 0; i < fill - packetBuf; ++i)
    Serial.write(packetBuf[i]);
}

static void initBluetooth (word baud) {
  Serial.begin(baud);
  delay(1000);
  Serial.print("$$$"); // enter command mode
  delay(100);
  Serial.println("C"); // connect to saved remote address
  delay(100);
  Serial.println("F,1"); // leave command mode
}

static void getSmaData () {
  memset(&payload, 0, sizeof payload);
  payload.yield = dailyYield();         // Wh
  payload.total = totalPower() / 1000;  // kWh
  payload.acw = acPowerNow();           // W
  dcVoltsNow(payload.dcv);              // V * 100
  dcPowerNow(payload.dcw);              // W
}

void setup () {
  rf12_initialize(15, RF12_868MHZ, 5);

  initBluetooth(19200);
  smaInitAndLogin(myAddrBT);
}

void loop () {
  if (sendTimer.poll(10000)) {
    getSmaData();

    rf12_sleep(RF12_WAKEUP);
    while (!rf12_canSend())
      rf12_recvDone();
    rf12_sendStart(0, &payload, sizeof payload);
    rf12_sendWait(2);
    rf12_sleep(RF12_SLEEP);
  }

  Sleepy::loseSomeTime(sendTimer.remaining());
}
