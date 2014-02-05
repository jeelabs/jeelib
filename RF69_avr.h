#include <avr/pgmspace.h>
#include <util/crc16.h>

#define ROM_UINT8       const prog_uint8_t
#define ROM_READ_UINT8  pgm_read_byte
#define ROM_DATA        PROGMEM

// ATmega168, ATmega328, etc.
// #define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8

#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define SPI_SS      2
#define SPI_MOSI    3
#define SPI_MISO    4
#define SPI_SCK     5

#define cs_pin      SS_BIT

struct PreventInterrupt {
    PreventInterrupt () { EIMSK &= ~ _BV(INT0); }
    ~PreventInterrupt () { EIMSK |= _BV(INT0); }
};

static void spiInit (void) {
    SS_PORT |= _BV(cs_pin);
    SS_DDR |= _BV(cs_pin);
    
    SPI_PORT |= _BV(SPI_SS);
    SPI_DDR |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
    SPI_DDR &= ~ _BV(SPI_MISO);
    
    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR |= _BV(SPI2X);
    
    // pinMode(RFM_IRQ, INPUT);
    // digitalWrite(RFM_IRQ, 1); // pull-up
}

static uint8_t spiTransfer (uint8_t cmd, uint8_t val) {
    SS_PORT &= ~ _BV(cs_pin);
    SPDR = cmd;
    while (!(SPSR & _BV(SPIF)))
        ;
    SPDR = val;
    while (!(SPSR & _BV(SPIF)))
        ;
    SS_PORT |= _BV(cs_pin);
    return SPDR;
}
