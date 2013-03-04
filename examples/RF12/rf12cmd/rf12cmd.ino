/// @dir rf12cmd
/// Command-based central node for RF12 packets.
// 2012-10-14 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <EmBencode.h>

#define RF12CMD_VERSION 1
#define BAND 868
#define GROUP 5

char embuf [100];
EmBdecode decoder (embuf, sizeof embuf);
EmBencode encoder;

void EmBencode::PushChar (char ch) {
  Serial.write(ch);
}

static uint8_t freqCode (int freq) {
  return freq == 433 ? RF12_433MHZ : freq == 868 ? RF12_868MHZ : RF12_915MHZ;
}

static void pushGreeting () {
  encoder.startList();
  encoder.push("hi");
  encoder.push("rf12cmd");
  encoder.push(RF12CMD_VERSION);
  encoder.push(sizeof embuf);
  encoder.endList();
}

static void pushBytes (uint8_t hdr, const void* ptr, uint8_t len) {
  Serial.flush();
  encoder.startList();
  encoder.push("rx");
  encoder.push(BAND);
  encoder.push(GROUP);
  encoder.push(hdr);
  encoder.push(ptr, len);
  encoder.endList();
}

static void pushError (const char* tag) {
  Serial.flush();
  encoder.startList();
  encoder.push(1);
  encoder.push("c?");
  if (tag != 0)
    encoder.push(tag);
  encoder.endList();
}

static bool do_tx () {
  if (decoder.nextToken() != EmBdecode::T_NUMBER) return false;
  int band = decoder.asNumber();
  if (decoder.nextToken() != EmBdecode::T_NUMBER) return false;
  uint8_t group = decoder.asNumber();
  if (decoder.nextToken() != EmBdecode::T_NUMBER) return false;
  char node = decoder.asNumber();
  if (decoder.nextToken() != EmBdecode::T_STRING) return false;

  rf12_initialize(node < 0 ? -node : 1, freqCode(band), group);

  // send the packet out on the specified frequency band and netgroup
  uint8_t len;
  const void* data = decoder.asString(&len);
  // rf12_sendStart(node < 0 ? 0 : node, data, len);
  rf12_sendNow(1, "\3", 1);
  rf12_sendWait(1);

  rf12_initialize(1, freqCode(BAND), GROUP);
  return true;
}

static void dispatch () {
  bool ok = false;
  const char* cmd = 0;

  if (decoder.nextToken() == EmBdecode::T_LIST &&
      decoder.nextToken() == EmBdecode::T_STRING) {
    cmd = decoder.asString();
    ok = strcmp(cmd, "tx") == 0 && do_tx();
  }
  
  decoder.reset();
  if (!ok)
    pushError(cmd);
}

void setup () {
  Serial.begin(115200);
  pushGreeting();
  while (Serial.read() >= 0)
    ;
  rf12_initialize(1, freqCode(BAND), GROUP);
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0)
    pushBytes(rf12_hdr, (const void*) rf12_data, rf12_len);

  if (Serial.available() && decoder.process(Serial.read()) > 0)
    dispatch();
}
