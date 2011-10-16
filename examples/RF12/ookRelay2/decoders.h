// Generalized decoder framework for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <Arduino.h>

class DecodeOOK {
protected:
    byte bits, flip, state, pos, data[25];

    // gets called once per incoming pulse with the width in us
    // return values: 0 = keep going, 1 = done, -1 = no match
    virtual char decode (word width) =0;
    
public:
    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK () { resetDecoder(); }

    bool nextPulse (word width) {
        if (state != DONE)
            switch (decode(width)) {
                case -1: resetDecoder(); break;
                case 1:  done(); break;
            }
        return isDone();
    }
    
    bool isDone () const { return state == DONE; }

    const byte* getData (byte& count) const {
        count = pos;
        return data; 
    }
    
    void resetDecoder () {
        bits = pos = flip = 0;
        state = UNKNOWN;
    }
    
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
    
    void done () {
        while (bits)
            gotBit(0); // padding
        state = DONE;
    }
};

// 433 MHz decoders

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

// The following three decoders were contributed bij Gijs van Duimen:
//    FlamingoDecoder = Flamingo FA15RF
//    SmokeDecoder = Flamingo FA12RF
//    ByronbellDecoder = Byron SX30T
// see http://www.domoticaforum.eu/viewtopic.php?f=17&t=4960&start=90#p51118
// there's some weirdness in this code, I've edited it a bit -jcw, 2011-10-16

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

class EMxDecoder : public DecodeOOK {
public:
    EMxDecoder () {}
    
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
typedef struct {
    char typecode;
    const char* name;
    DecodeOOK* decoder;
} DecoderInfo;
