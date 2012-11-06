/// Connect to SMA inverters via Bluetooth and read out some of its values.
// 2012-11-04 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// This code can be used in a normal Unix program or in an Arduino sketch.
// Include this header in a C++ file which defines getBytes() and emitFinal().
//
// Written from scratch, with lots of details gleaned from Stuart Pittaway's
// OSS project at https://github.com/stuartpittaway/nanodesmapvmonitor

static byte getByte (); // forward
static void emitFinal (); // forward

// use flash-based storage for some things when running on an ATmega
#ifdef JeeLib_h
#include <avr/pgmspace.h>
#define PFETCH(x) pgm_read_byte(x)
#else
#define PSTR(x) x
#define PFETCH(x) *(x)
#endif

byte check;
byte packetNum;
word fcsCheck;
byte* fill;
bool escaped;

byte packetBuf [150];
word packetLen;
byte srcAddr[6], destAddr[6], smaAddr[6];
const byte* myAddr;

byte fakeAddr[] = { 0x5C,0xAF,0xF0,0x1D,0x50,0x00 };
byte allffs[] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
char passwd[12] = "0000";

struct { byte src[6]; byte dest[6]; word cmd; } header;

#ifdef JeeLib_h
prog_uint16_t fcstab[] PROGMEM =
#else
word fcstab[] =
#endif
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static void fcsUpdate (byte b) {
#ifdef JeeLib_h
  fcsCheck = (fcsCheck >> 8) ^ pgm_read_word(&fcstab[(byte) fcsCheck ^ b]);
#else
  fcsCheck = (fcsCheck >> 8) ^ fcstab[(byte) fcsCheck ^ b];
#endif
}

static void getBytes (void* ptr, word len) {
  byte* p = (byte*) ptr;
  while (len-- > 0)
    *p++ = getByte();
}

static word getEscaped (void* ptr, int len) {
  byte* p = (byte*) ptr;
  byte last = 0;
  while (--len >= 0) {
    byte b = getByte();
    if (b == 0x7D) {
      b = getByte() ^ 0x20;
      --len;
    }
    *p++ = b;
  }
  return p - (byte*) ptr;
}

static word readPacket () {
  do
    check = 0;
  while (getByte() != 0x7E);

  struct { word len; byte check; } prefix;
  getBytes(&prefix, 3);
  byte pcheck = check;
  getBytes(&header, 14);
  
  if (pcheck != 0 || prefix.len >= sizeof packetBuf)
    return 0;

  packetLen = getEscaped(packetBuf, prefix.len - 18);

  // printf("\n  cmd %04X len %d plen %d\n", header.cmd, prefix.len, packetLen);
  return header.cmd;
}

static bool expectPacket (word cmd) {
  // printf("expect %04X\n", cmd);
  word rcmd;
  do {
    rcmd = readPacket();
    if (rcmd == cmd)
      return true;
  } while (rcmd != 0);
  return false;
}

static void emitOne (byte b) {
  if (escaped) {
    fcsUpdate(b);
    switch (b) {
      case 0x7D: case 0x7E: case 0x11: case 0x12: case 0x13:
        *fill++ = 0x7D;
        b ^= 0x20;
    }
  }
  *fill++ = b;
}

static void emitBytes (const void* ptr, byte len) {
  const byte* p = (const byte*) ptr;
  while (len-- > 0)
    emitOne(*p++);
}

static void emitInt (uint32_t data, byte len) {
  emitBytes(&data, len);
}

static void emitStart () {
  escaped = false;
  fcsCheck = ~0;
  fill = packetBuf;
  emitInt(0x7E, 4);
}

static void sendPacket (const char* fmt, ...) {
  emitStart();
  va_list ap;
  va_start(ap, fmt);

  while (PFETCH(fmt)) {
    char c = PFETCH(fmt++);
    switch (c) {

      case '$':
        switch (PFETCH(fmt++)) {
          case 'c': emitOne(va_arg(ap, int)); break;
          case 'i': emitInt(va_arg(ap, int), 2); break;
          case 'l': emitInt(va_arg(ap, long), 4); break;
          case 'm': emitBytes(myAddr, 6); break;
          case 's': emitBytes(smaAddr, 6); break;
          case 'f': emitBytes(allffs, 6); break;
          case 'z': emitInt(0L, 4); break;
          case 'x': emitInt(0x00020080L, 4); break;
          case 'p':
            for (byte i = 0; i < 12; ++i)
              emitOne((passwd[i] + 0x88) % 255);
            break;
        }
        break;

      case '/':
        emitInt(0x0001, 2);
        emitOne(0x7E);
        escaped = true;
        emitInt(0x656003FFL, 4);
        emitInt(va_arg(ap, int), 2);
        emitBytes(allffs, 6);
        emitInt(va_arg(ap, int), 2);
        emitBytes(fakeAddr, 6);
        emitOne(0x00);
        emitOne(va_arg(ap, int));
        emitInt(0L, 4);
        emitOne(packetNum);
        break;

      default:
        if (c >= '0') {
          byte b = (c > '9' ? c - 7 : c) << 4;
          c = PFETCH(fmt++);
          b += (c > '9' ? c - 7 : c) & 0x0F;
          emitOne(b);
        }
    }
  }

  if (escaped) {
    escaped = false;
    emitInt(~fcsCheck, 2);
    emitOne(0x7E);
  }

  va_end(ap);
  emitFinal();
}

static bool validPacket () {
  if (packetLen < 28 || packetBuf[27] != packetNum ||
      memcmp(packetBuf, "\x7E\xFF\x03\x60\x65", 5) != 0)
    return false;

  fcsCheck = ~0;
  for (word i = 1; i < packetLen - 3; ++i) 
    fcsUpdate(packetBuf[i]);
  return (word) ~fcsCheck == *(word*)(packetBuf+packetLen-3);
}

static void sendAndWait (const char* fmt, word a1, word a2, word a3) {
  do
    sendPacket(fmt, a1, a2, a3);
  while (!expectPacket(0x0001) || !validPacket());
  ++packetNum;
}

static byte smaInitAndLogin (const byte* myBtAddr) {
  myAddr = myBtAddr;
  if (!expectPacket(0x0002))
    return 1;
  memcpy(smaAddr, header.src, 6); // save the SMA's BT address
  sendPacket(PSTR("$m $s 020000047000 $c $z 01000000"), packetBuf[4]);
  if (!expectPacket(0x000A) || !expectPacket(0x0005))
    return 2;

  sendAndWait(PSTR("$m $f / $x 00 $z $z"), 0xA009, 0, 0);
  sendPacket(PSTR("$m $f / 800E01FDFFFFFFFFFF"), 0xA008, 0x0300, 0x03);
  ++packetNum;

  sendAndWait(PSTR("$m $f / 800C04FDFF0700000084030000AAAABBBB $z $p"),
                                                        0xA00E, 0x0100, 0x01);
  return 0; // ok
}

static void setInverterTime (uint32_t now) {
  sendPacket(PSTR("$m $s / 8C0A0200F0006D2300006D2300006D2300"
                  " $l $l $l $i 0000 $l 01000000"),
              0x0009, 0x0000, 0, now, now, now, 0, now);
  ++packetNum;
}

static word dailyYield (uint32_t* ptime =0) {
  sendAndWait(PSTR("$m $f / $x 5400222600FF222600"), 0xA009, 0, 0);
  if (ptime != 0)
    *ptime = *(uint32_t*)(packetBuf+45);
  if (*(word*)(packetBuf+42) != 0x2622)
    return 0;
  return *(word*)(packetBuf+49);
}

static uint32_t totalPower () {
  sendAndWait(PSTR("$m $s / $x 5400012600FF012600"), 0xA009, 0, 0);
  return *(uint32_t*)(packetBuf+49);
}

static word acPowerNow () {
  sendAndWait(PSTR("$m $f / $x 51003F2600FF3F26000E"), 0xA109, 0, 0);
  return *(word*)(packetBuf+49);
}

static byte dcVoltsNow (word* buf) {
  do
    sendPacket(PSTR("$m $s / 830002805300004500FFFF4500"), 0xE009, 0, 0);
  while (!expectPacket(0x0008));
  byte n = 0;
  for (word i = 41; i < packetLen - 3; i += 28) {
    word type = *(word*)(packetBuf+i+1);
    // printf("type %x\n", type);
    if (type == 0x451F)
      buf[n++] = *(word*)(packetBuf+i+8);
  }
  ++packetNum;
  return expectPacket(0x0001) ? n : 0;
}

static byte dcPowerNow (word* buf) {
  do
    sendPacket(PSTR("$m $s / 830002805300002500FFFF2500"), 0xE009, 0, 0);
  while (!expectPacket(0x0008));
  byte n = 0;
  for (word i = 41; i < packetLen - 3; i += 28) {
    word type = *(word*)(packetBuf+i+1);
    // printf("type %x\n", type);
    if (type == 0x251E)
      buf[n++] = *(word*)(packetBuf+i+8);
  }
  ++packetNum;
  return expectPacket(0x0001) ? n : 0;
}
