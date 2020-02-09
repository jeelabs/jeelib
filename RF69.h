#ifndef RF69_h
#define RF69_h

#define JEEPACKET16 80					// Loop limiter in RF69::interrupt_compat
#define JEEPACKET8 JEEPACKET16				// Loop limiter in RF69::interrupt_compat
// 					The above value is an estimate and needs fine tuning.
#define SALUSPACKET16  2500					// ditto

typedef struct {

    uint32_t rateInterval;	// Disabled by default, set this to 10000ms
    volatile uint32_t RSSIrestart;
    volatile uint32_t interruptCountRX;
    volatile uint32_t interruptCountTX;
    uint32_t syncMatch;
    uint32_t goodCRC;
	uint32_t discards;   // Count of good packets discarded
    uint32_t interpacketTS;
    uint32_t softDelay;
    
    volatile uint32_t rxLast;
    volatile uint32_t minGap;
    volatile uint32_t maxGap;
    volatile uint32_t debug;
    
    uint32_t cumRSSI[8];
    int32_t cumFEI[8];
//    int32_t cumAFC[8];
//    uint32_t cumLNA[8];
    uint16_t cumCount[8];
    uint16_t cumZeros[8];
	uint16_t RssiToSync;
	uint16_t RssiToSyncLimit;	//Count of loops after RSSI before a missed sync is triggered
	uint16_t rssiZero;
    volatile uint16_t rtpMin;    
	volatile uint16_t rtpMax;
	volatile uint16_t intRXFIFO;
	volatile uint16_t TXIDLECount;
	uint8_t modeError;    
	uint8_t	configThreshold;
    uint8_t rssiThreshold;
    uint8_t rssi;
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
	uint8_t lastPay0;
	uint8_t	txPower;
	uint8_t changed;
	uint8_t ConfigFlags;

	} RF_API;
	extern RF_API rfapi;	// Info interchange area
    
namespace RF69 {
    extern uint32_t frf;
    extern uint8_t  group;
    extern uint8_t  node;
    extern uint8_t microOffset;
    extern uint8_t  rssi;
    extern uint8_t  rssiDelay;
    extern uint8_t lastState;
    extern uint8_t REGIRQFLAGS1;
    extern int16_t  afc;
    extern int16_t  fei;
    extern uint8_t  lna;
    extern uint16_t interruptCountRX;
    extern uint16_t interruptCountTX;
    extern uint16_t rxP;
    extern uint16_t txP;
    extern uint16_t discards;
    extern uint16_t unexpected;
    extern uint8_t  unexpectedFSM;
    extern uint8_t  unexpectedIRQFLAGS2;
    extern uint8_t  unexpectedMode;
    extern uint16_t byteCount;
    extern uint16_t underrun;
    extern uint8_t  present;
    extern uint16_t pcIntCount;
    extern uint8_t  pcIntBits;
    extern int8_t   payloadLen;
    extern uint16_t badLen;
    extern uint16_t packetShort;
    extern uint8_t  IRQFLAGS2;
    extern uint8_t  DIOMAPPING1;

    void setFrequency (uint32_t freq);
    uint8_t canSend (uint8_t clearAir);
    bool sending ();
    void sleep (bool off);
    uint8_t control(uint8_t cmd, uint8_t val);
    uint8_t radioIndex(uint8_t index, uint8_t val);    
    int8_t readTemperature(int8_t usercal);
    uint8_t* SPI_pins();  // {OPTIMIZE_SPI, PINCHG_IRQ, RF69_COMPAT, RFM_IRQ, 
                          //  SPI_SS, SPI_MOSI, SPI_MISO, SPI_SCK }
    uint8_t currentRSSI();     
    void configure_compat ();
    uint16_t recvDone_compat (uint8_t* buf);
/// Call this to skip transmission of specific bytes in rf12_buf
/// Default value 2 skips the Jeelib header enabling non-Jeelib FSK packets 
    void skip_hdr (uint8_t skip = 2);
    void fix_len (uint8_t skip = 16);
    void sendStart_compat (uint8_t hdr, const void* ptr, uint8_t len);
    void RSSIinterrupt();
    void interrupt_compat(uint8_t RSSI_INTERRUPT);
    void interruptTX();
    void interrupt_spare();
}

#endif
