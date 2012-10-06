/// @file
/// Generalized decoder framework for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <Arduino.h>
#include <util/crc16.h>

/// This is the general base class for implementing OOK decoders.
class DecodeOOK {
protected:
    byte bits, flip, state, pos, data[25];
    // the following fields are used to deal with duplicate packets
    word lastCrc, lastTime;
    byte repeats, minGap, minCount;

    // gets called once per incoming pulse with the width in us
    // return values: 0 = keep going, 1 = done, -1 = no match
    virtual char decode (word width) =0;
    
    // add one bit to the packet data buffer
    void gotBit (char value) {
        byte *ptr = data + pos;
        *ptr = (*ptr >> 1) | (value << 7);

        if (++bits >= 8) {
            bits = 0;
            if (++pos >= sizeof data) {
                resetDecoder();
                return;
            }
        }
        
        state = OK;
    }
    
    // store a bit using Manchester encoding
    void manchester (char value) {
        flip ^= value; // manchester code, long pulse flips the bit
        gotBit(flip);
    }
    
    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte max =0) {
        // align bits
        if (bits != 0) {
            data[pos] >>= 8 - bits;
            for (byte i = 0; i < pos; ++i)
                data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
            bits = 0;
        }
        // optionally shift bytes down if there are too many of 'em
        if (max > 0 && pos > max) {
            byte n = pos - max;
            pos = max;
            for (byte i = 0; i < pos; ++i)
                data[i] = data[i+n];
        }
    }
    
    void reverseBits () {
        for (byte i = 0; i < pos; ++i) {
            byte b = data[i];
            for (byte j = 0; j < 8; ++j) {
                data[i] = (data[i] << 1) | (b & 1);
                b >>= 1;
            }
        }
    }
    
    void reverseNibbles () {
        for (byte i = 0; i < pos; ++i)
            data[i] = (data[i] << 4) | (data[i] >> 4);
    }
    
    bool checkRepeats () {
        // calculate the checksum over the current packet
        word crc = ~0;
        for (byte i = 0; i < pos; ++i)
            crc = _crc16_update(crc, data[i]);
        // how long was it since the last decoded packet
        word now = millis() / 100; // tenths of seconds
        word since = now - lastTime;
        // if different crc or too long ago, this cannot be a repeated packet
        if (crc != lastCrc || since > minGap)
            repeats = 0;
        // save last values and decide whether to report this as a new packet
        lastCrc = crc;
        lastTime = now;
        return repeats++ == minCount;
    }

public:
    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK (byte gap =5, byte count =0) 
        : lastCrc (0), lastTime (0), repeats (0), minGap (gap), minCount (count)
        { resetDecoder(); }
        
    bool nextPulse (word width) {
        if (state != DONE)
            switch (decode(width)) {
                case -1: // decoding failed
                    resetDecoder();
                    break;
                case 1: // decoding finished
                    while (bits)
                        gotBit(0); // padding
                    state = checkRepeats() ? DONE : UNKNOWN;
                    break;
            }
        return state == DONE;
    }
    
    const byte* getData (byte& count) const {
        count = pos;
        return data; 
    }
    
    void resetDecoder () {
        bits = pos = flip = 0;
        state = UNKNOWN;
    }
};

// 433 MHz decoders

/// OOK decoder for Oregon Scientific devices.
class OregonDecoder : public DecodeOOK {
public:
    OregonDecoder () {}
    
    virtual char decode (word width) {
        if (200 <= width && width < 1200) {
            byte w = width >= 700;
            switch (state) {
                case UNKNOWN:
                    if (w == 0)
                        ++flip;
                    else if (10 <= flip && flip <= 50) {
                        flip = 1;
                        manchester(1);
                    } else
                        return -1;
                    break;
                case OK:
                    if (w == 0)
                        state = T0;
                    else
                        manchester(1);
                    break;
                case T0:
                    if (w == 0)
                        manchester(0);
                    else
                        return -1;
                    break;
            }
            return 0;
        }
        if (width >= 2500 && pos >= 9) 
            return 1;
        return -1;
    }
};

/// OOK decoder for Cresta devices.
class CrestaDecoder : public DecodeOOK {
  // http://members.upc.nl/m.beukelaar/Crestaprotocol.pdf
public:
    CrestaDecoder () {}
    
    virtual char decode (word width) {
        if (200 <= width && width < 1300) {
            byte w = width >= 750;
            switch (state) {
                case UNKNOWN:
                    if (w == 1)
                        ++flip;
                    else if (2 <= flip && flip <= 10)
                        state = T0;
                    else
                        return -1;
                    break;
                case OK:
                    if (w == 0)
                        state = T0;
                    else
                        gotBit(1);
                    break;
                case T0:
                    if (w == 0)
                        gotBit(0);
                    else
                        return -1;
                    break;
            }
            return 0;
        }
        if (width >= 2500 && pos >= 7) 
            return 1;
        return -1;
    }
};

/// OOK decoder for Klik-Aan-Klik-Uit devices.
class KakuDecoder : public DecodeOOK {
public:
    KakuDecoder () {}
    
    virtual char decode (word width) {
        if (180 <= width && width < 450 || 950 <= width && width < 1250) {
            byte w = width >= 700;
            switch (state) {
                case UNKNOWN:
                case OK:
                    if (w == 0)
                        state = T0;
                    else
                        return -1;
                    break;
                case T0:
                    if (w)
                        state = T1;
                    else
                        return -1;
                    break;
                case T1:
                    state += w + 1;
                    break;
                case T2:
                    if (w)
                        gotBit(0);
                    else
                        return -1;
                    break;
                case T3:
                    if (w == 0)
                        gotBit(1);
                    else
                        return -1;
                    break;
            }
            return 0;
        }
        if (width >= 2500 && 8 * pos + bits == 12) {
            for (byte i = 0; i < 4; ++i)
                gotBit(0);
            alignTail(2);
            return 1;
        }
        return -1;
    }
};

/// OOK decoder for Klik-Aan-Klik-Uit type A devices.
class KakuADecoder : public DecodeOOK {
    enum { Pu, P1, P5, P10 }; //pulsetypes: Pu = unknown, P1 = between 100 and 600uS, P5 = between 800 and 1800uS, P10 = between 2 and 3 mS
    uint8_t backBuffer[4]; //we need to keep a 4 bit history
    byte pulse;  

  public:
    KakuADecoder () {
      clearBackBuffer();
    }
    
    void clearBackBuffer()
    {
      backBuffer[0] = Pu;
      backBuffer[1] = Pu;
      backBuffer[2] = Pu;
      backBuffer[3] = Pu;
    }
    
    virtual char decode (word width) {
        if ((width >= 100) && (width <= 600))
          pulse = P1;
        else if ((width >=800) && (width <= 1800))
          pulse = P5;
        else if ((width >=2000) && (width <= 3000))
          pulse = P10;
        else 
        {
          clearBackBuffer();//out-of-protocol pulsewidth, abort;
          return -1; //reset decoder
        }
        
        backBuffer[3] = backBuffer[2];
        backBuffer[2] = backBuffer[1];
        backBuffer[1] = backBuffer[0];
        backBuffer[0] = pulse;
        
        switch(state)
        {
          case UNKNOWN:
          if( backBuffer[2] == P1 && backBuffer[1] == P10 && backBuffer[0] == P1 ) //received start/sync signal
          {
            state = T0;
            clearBackBuffer();
          }
          break;
          case OK: //returning after receiving a good bit
          case T0:
          if( pulse == P10 ) //received start/sync signal
          {
            clearBackBuffer();
            return -1; //reset decoder
          } 
          if( backBuffer[3] != Pu ) //depending on the preceding pulsetypes we received a 1 or 0
          {
            if ( (backBuffer[3] == P5) && (backBuffer[2] == P1) && (backBuffer[1] == P1) && (backBuffer[0] == P1))
              gotBit(1);
            else
            if ( (backBuffer[3] == P1) && (backBuffer[2] == P1) && (backBuffer[1] == P5) && (backBuffer[0] == P1))
              gotBit(0);
            else
            { 
              state = UNKNOWN;
              break;
            }
            clearBackBuffer();            
            if( pos >= 4 ) //we expect 4 bytes
              return 1;
          }
          break;
          default:
          break;
        }
        return 0;
    }
};

/// OOK decoder for X11 over RF devices.
class XrfDecoder : public DecodeOOK {
public:
    XrfDecoder () {}
    
    // see also http://davehouston.net/rf.htm
    virtual char decode (word width) {
        if (width > 2000 && pos >= 4)
            return 1;
        if (width > 5000)
            return -1;
        if (width > 4000 && state == UNKNOWN) {
            state = OK;
            return 0;
        }
        if (350 <= width && width < 1800) {
            byte w = width >= 720;
            switch (state) {
                case OK:
                    if (w == 0)
                        state = T0;
                    else
                        return -1;
                    break;
                case T0:
                    gotBit(w);
                    break;
            }
            return 0;
        }
        return -1;
    }
};

/// OOK decoder for FS20 type HEZ devices.
class HezDecoder : public DecodeOOK {
public:
    HezDecoder () {}
    
    // see also http://homeeasyhacking.wikia.com/wiki/Home_Easy_Hacking_Wiki
    virtual char decode (word width) {
        if (200 <= width && width < 1200) {
            gotBit(width >= 600);
            return 0;
        }
        if (width >= 5000 && pos >= 5) {
            for (byte i = 0; i < 6; ++i)
                gotBit(0);
            alignTail(7); // keep last 56 bits
            return 1;
        }
        return -1;
    }
};

/// OOK decoder for Elro devices.
class ElroDecoder : public DecodeOOK {
public:
    ElroDecoder () {}
    
    virtual char decode (word width) {
        if (50 <= width && width < 600) {
            byte w = (width - 40) / 190; // 40 <= 0 < 230 <= 1 < 420 <= 2 < 610
            switch (state) {
                case UNKNOWN:
                case OK:
                    if (w == 0)
                        state = T0;
                    else if (w == 2)
                        state = T2;
                    break;
                case T0:
                case T2:
                    if (w == 1)
                        ++state;
                    else
                        return -1;
                    break;
                case T1:
                    if (w == 0) { // sync pattern has 0-1-0-1 patterns
                        resetDecoder();
                        break;
                    }
                    if (w != 2)
                        return -1;
                    gotBit(0);
                    break;
                case T3:
                    if (w != 0)
                        return -1;
                    gotBit(1);
                    break;
            }
            return 0;
        }
        if (pos >= 11)
            return 1;
        return -1;
    }
};

// The following three decoders were contributed bij Gijs van Duimen:
//    FlamingoDecoder = Flamingo FA15RF
//    SmokeDecoder = Flamingo FA12RF
//    ByronbellDecoder = Byron SX30T
// see http://www.domoticaforum.eu/viewtopic.php?f=17&t=4960&start=90#p51118
// there's some weirdness in this code, I've edited it a bit -jcw, 2011-10-16

/// OOK decoder for Flamingo devices.
class FlamingoDecoder : public DecodeOOK {
public:
    FlamingoDecoder () {}
     
    virtual char decode (word width) {
        if ((width > 740 && width < 780) || (width > 2650 && width < 2750) ||
             (width > 810 && width < 950) || (width > 1040 && width < 1450)) {
            gotBit(width >= 950);
            return 0;
        }
        // if (pos >= 4 && data[0] == 84 && data[1] == 85 &&
        //                 data[2] == 85 && data[3] == 85)
        if (pos >= 4)
            return 1; 
        return -1;
    }
};

/// OOK decoder for Flamingo smoke devices.
class SmokeDecoder : public DecodeOOK {
public:
    SmokeDecoder () {}
     
    virtual char decode (word width) {
        if (width > 20000 && width < 21000 || width > 6900 && width < 7000 ||
            width > 6500 && width < 6800) {
            gotBit(1);
            // if (width > 3000 && width < 4000)
            //     byte w = width < 100;
            return 0;
        }
        if (pos >= 4)
            return pos = bits = 1; 
        return -1;
    }
};

/// OOK decoder for Byronbell devices.
class ByronbellDecoder : public DecodeOOK {
public:
    ByronbellDecoder () {}
     
    virtual char decode (word width) {
        if (660 < width && width < 715 || 5100 < width && width < 5400) {
            gotBit(width > 1000);
            return 0;
        }
        if (pos >= 8)
            return pos = bits = 1; 
        return -1;
    }
};

// 868 MHz decoders

/// OOK decoder for Visonic devices.
class VisonicDecoder : public DecodeOOK {
public:
    VisonicDecoder () {}
    
    virtual char decode (word width) {
        if (200 <= width && width < 1000) {
            byte w = width >= 600;
            switch (state) {
                case UNKNOWN:
                case OK:
                    state = w == 0 ? T0 : T1;
                    break;
                case T0:
                    gotBit(!w);
                    if (w)
                        return 0;
                    break;
                case T1:
                    gotBit(!w);
                    if (!w)
                        return 0;
                    break;
            }
            // sync error, flip all the preceding bits to resync
            for (byte i = 0; i <= pos; ++i)
                data[i] ^= 0xFF; 
        } else if (width >= 2500 && 8 * pos + bits >= 36 && state == OK) {
            for (byte i = 0; i < 4; ++i)
                gotBit(0);
            alignTail(5); // keep last 40 bits
            // only report valid packets
            byte b = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
            if ((b & 0xF) == (b >> 4))
                return 1;
        } else
            return -1;
        return 0;
    }
};

/// OOK decoder for FS20 type EM devices.
class EMxDecoder : public DecodeOOK {
public:
    EMxDecoder () : DecodeOOK (30) {} // ignore packets repeated within 3 sec
    
    // see also http://fhz4linux.info/tiki-index.php?page=EM+Protocol
    virtual char decode (word width) {
        if (200 <= width && width < 1000) {
            byte w = width >= 600;
            switch (state) {
                case UNKNOWN:
                    if (w == 0)
                        ++flip;
                    else if (flip > 20)
                        state = OK;
                    else
                        return -1;
                    break;
                case OK:
                    if (w == 0)
                        state = T0;
                    else
                        return -1;
                    break;
                case T0:
                    gotBit(w);
                    break;
            }
        } else if (width >= 1500 && pos >= 9)
            return 1;
        else
            return -1;
        return 0;
    }
};

/// OOK decoder for FS20 type KS devices.
class KSxDecoder : public DecodeOOK {
public:
    KSxDecoder () {}
    
    // see also http://www.dc3yc.homepage.t-online.de/protocol.htm
    virtual char decode (word width) {
        if (200 <= width && width < 1000) {
            byte w = width >= 600;
            switch (state) {
                case UNKNOWN:
                    gotBit(w);
                    bits = pos = 0;
                    if (data[0] != 0x95)
                        state = UNKNOWN;
                    break;
                case OK:
                    state = w == 0 ? T0 : T1;
                    break;
                case T0:
                    gotBit(1);
                    if (!w)
                        return -1;
                    break;
                case T1:
                    gotBit(0);
                    if (w)
                        return -1;
                    break;
            }
        } else if (width >= 1500 && pos >= 6) 
            return 1;
        else
            return -1;
        return 0;
    }
};

/// OOK decoder for FS20 type FS devices.
class FSxDecoder : public DecodeOOK {
public:
    FSxDecoder () {}
    
    // see also http://fhz4linux.info/tiki-index.php?page=FS20%20Protocol
    virtual char decode (word width) {
        if (300 <= width && width < 775) {
            byte w = width >= 500;
            switch (state) {
                case UNKNOWN:
                    if (w == 0)
                        ++flip;
                    else if (flip > 20)
                        state = T1;
                    else
                        return -1;
                    break;
                case OK:
                    state = w == 0 ? T0 : T1;
                    break;
                case T0:
                    gotBit(0);
                    if (w)
                        return -1;
                    break;
                case T1:
                    gotBit(1);
                    if (!w)
                        return -1;
                    break;
            }
        } else if (width >= 1500 && pos >= 5)
            return 1;
        else
            return -1;
        return 0;
    }
};

// Dumb Arduino IDE pre-processing bug - can't put this in the main source file!
/// Structure used to defined each entry in the decoder table.
typedef struct {
    char typecode;
    const char* name;
    DecodeOOK* decoder;
} DecoderInfo;
