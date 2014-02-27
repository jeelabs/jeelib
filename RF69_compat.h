// Include this file instead of RF12.h to use a RFM69 wireless radio module
// in compatibility mode, i.e. as if it were an RFM12, or insert this line
// before including the <JeeLib.h> header in your code: #define RF69_COMPAT 1

#ifndef RF69_compat_h
#define RF69_compat_h

#ifdef RF12_h
#error This file must be included BEFORE the "RF12.h" header file!
#endif

#define rf12_crc            rf69_crc
#define rf12_buf            rf69_buf
#define rf12_seq            rf69_seq
                            
#define rf12_set_cs         rf69_set_cs
#define rf12_spiInit        rf69_spiInit
#define rf12_initialize     rf69_initialize
#define rf12_config         rf69_config
#define rf12_configSilent   rf69_configSilent
#define rf12_recvDone       rf69_recvDone
#define rf12_canSend        rf69_canSend
#define rf12_sendStart      rf69_sendStart
#define rf12_sendNow        rf69_sendNow
#define rf12_sendWait       rf69_sendWait
#define rf12_onOff          rf69_onOff
#define rf12_sleep          rf69_sleep
#define rf12_lowbat         rf69_lowbat
#define rf12_easyInit       rf69_easyInit
#define rf12_easyPoll       rf69_easyPoll
#define rf12_easySend       rf69_easySend
#define rf12_encrypt        rf69_encrypt
#define rf12_control        rf69_control

#endif
