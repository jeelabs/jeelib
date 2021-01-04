#ifndef RFAPI_h
#define RFAPI_h
#include <stdint.h>
struct rfapi
{
    uint32_t rateInterval;	// Disabled by default, set this to 10000ms
    uint32_t RSSIrestart;
    uint32_t syncMatch;
    uint32_t goodCRC;
	uint32_t discards;   // Count of good packets discarded
    uint32_t interpacketTS;
    
	volatile uint32_t interruptCountRX;
	volatile uint32_t interruptCountTX;
    volatile uint32_t rxLast;
    volatile uint32_t minGap;
    volatile uint32_t maxGap;
    
    uint32_t cumRSSI[8];
    int32_t cumFEI[8];
    uint16_t cumCount[8];
    uint16_t cumZeros[8];
	uint16_t RssiToSync;
	uint16_t RssiToSyncLimit;	// Count of loops after RSSI before a missed sync is triggered
	uint16_t rssiZero;
	uint16_t fei;
    volatile uint16_t rtpMin;    
	volatile uint16_t rtpMax;
	volatile uint16_t intRXFIFO;
	volatile uint16_t noiseTail;
	volatile uint16_t debug;
	uint8_t modeError;    
	uint8_t	configThreshold;
    uint8_t rssiThreshold;
    uint8_t rssi;
	uint8_t lna;
    uint8_t	noiseFloorMin;
    uint8_t	noiseFloorMax;
    uint8_t	sendRSSI;
    uint8_t	setmode;
    uint8_t	irqflags1;
    uint8_t	mode;
	uint8_t	len;
	uint8_t rxfill;
	uint8_t rxdone;
	uint8_t	lastHdr;
	uint8_t	lastLen;
	uint8_t advisedLen;
	uint8_t lastPay0;
	uint8_t	txPower;
	uint8_t changed;
	uint8_t ConfigFlags;

};	// Info interchange area
typedef struct rfapi rfAPI;
#endif
 