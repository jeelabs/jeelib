#include <avr/interrupt.h>
#include <util/crc16.h>

// prog_uint8_t appears to be deprecated in avr libc, this resolves it for now
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#define ROM_UINT8       const prog_uint8_t
#define ROM_READ_UINT8  pgm_read_byte
#define ROM_DATA        PROGMEM

#define IRQ_ENABLE      sei()

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      0

#define SPI_SS      53    // PB0, pin 19
#define SPI_MOSI    51    // PB2, pin 21
#define SPI_MISO    50    // PB3, pin 22
#define SPI_SCK     52    // PB1, pin 20

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#elif defined(__AVR_ATmega644P__)

#define RFM_IRQ     10
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      4

#define SPI_SS      4
#define SPI_MOSI    5
#define SPI_MISO    6
#define SPI_SCK     7

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny44__)

#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      1

#define SPI_SS      1     // PB1, pin 3
#define SPI_MISO    4     // PA6, pin 7
#define SPI_MOSI    5     // PA5, pin 8
#define SPI_SCK     6     // PA4, pin 9

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS);
    PORTA |= _BV(SPI_SS);
    DDRA |= _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#elif defined(__AVR_ATmega32U4__) //Arduino Leonardo 

#define RFM_IRQ     3	  // PD0, INT0, Digital3 
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      6	  // Dig10, PB6

#define SPI_SS      17    // PB0, pin 8, Digital17
#define SPI_MISO    14    // PB3, pin 11, Digital14
#define SPI_MOSI    16    // PB2, pin 10, Digital16
#define SPI_SCK     15    // PB1, pin 9, Digital15

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#else // ATmega168, ATmega328, etc.

// #define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_SS      2     // PB0
#define SPI_MOSI    3     // PB1
#define SPI_MISO    4     // PB2
#define SPI_SCK     5     // PB3

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT);
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

#endif

#ifndef EIMSK
#define EIMSK GIMSK // ATtiny
#endif

struct PreventInterrupt {
    PreventInterrupt () { EIMSK &= ~ _BV(INT0); }
    ~PreventInterrupt () { EIMSK |= _BV(INT0); }
};

static void spiInit (void) {
    spiConfigPins();
    
#ifdef SPCR    
    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR |= _BV(SPI2X);
#else
    USICR = _BV(USIWM0); // ATtiny
#endif    
    
    // pinMode(RFM_IRQ, INPUT);
    // digitalWrite(RFM_IRQ, 1); // pull-up
}

static uint8_t spiTransferByte (uint8_t out) {
#ifdef SPDR
    SPDR = out;
    while (!(SPSR & _BV(SPIF)))
        ;
    return SPDR;
#else
    USIDR = out; // ATtiny
    uint8_t v1 = _BV(USIWM0) | _BV(USITC);
    uint8_t v2 = _BV(USIWM0) | _BV(USITC) | _BV(USICLK);
    for (uint8_t i = 0; i < 8; ++i) {
        USICR = v1;
        USICR = v2;
    }
    return USIDR;
#endif
}

static uint8_t spiTransfer (uint8_t cmd, uint8_t val) {
    SS_PORT &= ~ _BV(SS_BIT);
    spiTransferByte(cmd);
    uint8_t in = spiTransferByte(val);
    SS_PORT |= _BV(SS_BIT);
    return in;
}
