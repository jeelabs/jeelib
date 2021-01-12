/// @dir roomNode
/// A version of Room Node using Airwick PIR with a couple of tweaks. JOH 2017/06/02
/// New version of the Room Node (derived from rooms.pde).
// 2010-10-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// see http://jeelabs.org/2010/10/20/new-roomnode-code/
// and http://jeelabs.org/2010/10/21/reporting-motion/

// The complexity in the code below comes from the fact that newly detected PIR
// motion needs to be reported as soon as possible, but only once, while all the
// other sensor values are being collected and averaged in a more regular cycle.
///////////////////////////////////////////////////////////////////////////////

#define RF69_COMPAT      0	 // define this to use the RF69 driver i.s.o. RF12 
///                          // The above flag must be set similarly in RF12.cpp
///                          // and RF69_avr.h
#define BME280_PORT  1   // defined if BME280 is connected to I2C
//#define BMP280_PORT  1   // defined if BME280 is connected to I2C

#include <JeeLib.h>
#include "RFAPI.h"		// Define
rfAPI rfapi;			// Declare
#include <PortsSHT11.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/crc16.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#if BME280_PORT
	#include <Adafruit_BME280.h>
#elif BMP280_PORT
	#include <Adafruit_BMP280.h>
#endif

uint8_t resetFlags __attribute__ ((section(".noinit")));
void resetFlagsInit(void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
    // save the reset flags passed from the bootloader
    __asm__ __volatile__ ("mov %0, r2\n" : "=r" (resetFlags) :);
}

#define crc_update      _crc16_update
#define BMX280_ADDRESS	0x76

#define SERIAL  0   // set to 1 to also report readings on the serial port
#define DEBUG   0   // set to 1 to display each loop() run and PIR trigger

// #define SHT11_PORT  1   // defined if SHT11 is connected to a port
//	#define HYT131_PORT 1   // defined if HYT131 is connected to a port
#define LDR_PORT    4   // defined if LDR is connected to a port's AIO pin
#define PIR_PORT    4   // defined if PIR is connected to a port's DIO pin

//#define RETRY_PERIOD    20  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     1   // maximum number of times to try transmission
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define SMOOTH          3   // smoothing factor used for running averages

#define ADC_CALIBRATE	0

#if F_CPU == 8000000UL
	#define IDLESPEED		4
	#define RADIOSPEED		1
#else
	#define IDLESPEED		4
	#define RADIOSPEED		2
#endif

#define SETTINGS_EEPROM_ADDR ((uint8_t*) 0x00)

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 0
#define NOP __asm__ __volatile__ ("nop\n\t")

// The scheduler makes it easy to perform various tasks at various times:

enum { MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

#if BME280_PORT
	Adafruit_BME280 bme; // I2C
#elif BMP280_PORT
	Adafruit_BMP280 bmp; // I2C
#endif
#if SERIAL
static void showString (PGM_P s); // forward declaration
#endif
// Other variables used in various places in the code:

static byte reportCount;    // count up until next report, i.e. packet send
static byte myNodeID;       // node ID used for this unit
static uint16_t measureCount;

// This defines the structure of the packets which get sent out by wireless:

#define BASIC_PAYLOADLENGTH		14
#define TIMEOUT_PAYLOADLENGTH 	17
#define EXTENDED_PAYLOADLENGTH 	21
static byte payloadLength;

struct {					//0		Offset, node #
	byte command;			//1		ACK command return field
    byte missedACK	:4;		//2
    byte attempts	:4;		//2		Transmission attempts
    byte badCRC		:4;		//3
	byte sequence	:4;		//3		Packet sequence count
//    byte moved 		:1;  	//3		motion detector: 0..1
//    byte lobat 		:1;  	//3		supply voltage dropped under 3.1V: 0..1
//	byte spare2		:2;		//3    
#if BME280_PORT || BMP280_PORT
    uint32_t pressure:24;	//4&5&6
#endif
    byte light;     		//7		light sensor: 0..255
    unsigned int humi:16;	//8&9	humidity: 0..100.00
    int temp:16; 			//10&11	temperature: -5000..+5000 (hundredths)
    byte vcc;				//12	Bandgap battery voltage
    uint8_t rssiThreshold;	//13
    uint8_t sendingPower;	//14	Power applied to transmission
    uint8_t lna;			//15
    uint8_t inboundRssi;	//16	Measured RSSI of the received Ack
    uint16_t	fei;		//17&18
	uint8_t rebootCode;		//19
    uint8_t powerSeenAs;	//20	Received power of a transmission as reported by a remote node
    byte ack_delay;			//21
    byte message[ (RF12_MAXDATA - EXTENDED_PAYLOADLENGTH) ];
} payload;

typedef struct {
    byte start;
    uint16_t MEASURE_PERIOD;
    uint16_t REPORT_EVERY;
    bool MEASURE :1;
    bool REPORT :1;
    bool changedLight :1;
    bool changedHumi :1;
    bool changedTemp :1;
    bool spareOne :1;
    bool spareTwo :1;
    bool spareThree :1;
    uint8_t lowVcc;
    int8_t	ackBounds;
    word crc;
} eeprom;
static eeprom settings;

int		lastTemp;
byte	firstTime = true;
byte 	lastLight;
byte	lastHumi;
byte	ackSW = RF12_HDR_ACK;
int8_t	ackPacer = 30;	// First few packets all with Ack to tune Threshold & TX Power
bool	changed;
bool	rebootRequested;
// Conditional code, depending on which sensors are connected and how:

#if SHT11_PORT
    SHT11 sht11 (SHT11_PORT);
#endif

#if HYT131_PORT
    PortI2C hyti2cport (HYT131_PORT);
    HYT131 hyt131 (hyti2cport);
#endif

#if LDR_PORT
    Port ldr (LDR_PORT);
#endif

#if PIR_PORT
    #define PIR_HOLD_TIME   30  // hold PIR value this many seconds after change
    #define PIR_PULLUP      0   // set to one to pull-up the PIR input pin
    #define PIR_INVERTED    1   // 0 or 1, to match PIR reporting high or low
    
    /// Interface to a Passive Infrared motion sensor.
    class PIR : public Port {
        volatile byte value, changed;
        volatile uint32_t lastOn;
    public:
        PIR (byte portnum)
            : Port (portnum), value (0), changed (0), lastOn (0) {}

        // this code is called from the pin-change interrupt handler
        void poll() {
            // see http://talk.jeelabs.net/topic/811#post-4734 for PIR_INVERTED
            byte pin = digiRead() ^ PIR_INVERTED;
            // if the pin just went on, then set the changed flag to report it
            if (pin) {
                if (!state())
                    changed = 1;
                lastOn = millis();
            }
            value = pin;
        }

        // state is true if curr value is still on or if it was on recently
        byte state() const {
            byte f = value;
            if (lastOn > 0)
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    if (millis() - lastOn < (uint32_t)(1000 * PIR_HOLD_TIME))
                        f = 1;
                }
            return f;
        }

        // return true if there is new motion to report
        byte triggered() {
            byte f = changed;
            changed = 0;
            return f;
        }
    };

    PIR pir (PIR_PORT);
    
	volatile bool maskPCINT = true;
	volatile unsigned int countPCINT;
    // the PIR signal comes in via a pin-change interrupt
    ISR(PCINT2_vect) {
    	countPCINT++; 
    	if (!maskPCINT) {
    		pir.poll(); 
    	}
    }
#endif

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

volatile bool adcDone;

ISR(ADC_vect) { adcDone = true; }

static byte vccRead (byte count =4) {
  set_sleep_mode(SLEEP_MODE_ADC);
  ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
  bitSet(ADCSRA, ADIE);
  while (count-- > 0) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
  }
  bitClear(ADCSRA, ADIE);  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1023U) / (ADC + 1) - ADC_CALIBRATE;
}

void clock_prescale(uint8_t factor)
{
// if (factor > 8) factor = 8;
   CLKPR = (1 << CLKPCE);
   CLKPR = factor;
}

// utility code to perform simple smoothing as a running average
static int smoothedAverage(int prev, int next, byte firstTime =0) {
    if (firstTime)
        return next;
    return ((SMOOTH - 1) * prev + next + SMOOTH / 2) / SMOOTH;
}

// spend a little time in power down mode while the SHT11 does a measurement
static void shtDelay () {
    Sleepy::loseSomeTime(32); // must wait at least 20 ms
}

// readout all the sensors and other values
static void doMeasure() {

    scheduler.timer(MEASURE, settings.MEASURE_PERIOD);

    #if SERIAL || DEBUG
//	Serial.println("doMeasure"); serialFlush();
	#endif
#if !RF69_COMPAT
//    payload.lobat = 0;//rf12_lowbat();
#endif
    payload.vcc = vccRead();
    
	#if SHT11_PORT
		#ifndef __AVR_ATtiny84__
        sht11.measure(SHT11::HUMI, shtDelay);        
        sht11.measure(SHT11::TEMP, shtDelay);
        float h, t;
        sht11.calculate(h, t);
        int humi = h, temp = 10 * t;
		#else
        //XXX TINY!
        int humi = 50, temp = 25;
		#endif
        payload.humi = smoothedAverage(payload.humi, humi, firstTime);
        payload.temp = smoothedAverage(payload.temp, temp, firstTime);
    #endif
    #if HYT131_PORT
        int humi, temp;
        hyt131.reading(temp, humi);
        payload.humi = smoothedAverage(payload.humi, humi/10, firstTime);
        payload.temp = smoothedAverage(payload.temp, temp, firstTime);
    #endif
    
	#if BME280_PORT
    	bme.setSampling(Adafruit_BME280::MODE_FORCED,
			Adafruit_BME280::SAMPLING_X1, // temperature
            Adafruit_BME280::SAMPLING_X1, // pressure
            Adafruit_BME280::SAMPLING_X1, // humidity
            Adafruit_BME280::FILTER_OFF   );
    	// Only needed in forced mode! In normal mode, you can remove the next line.
    	bme.takeForcedMeasurement();// has no effect in normal mode
    	Sleepy::loseSomeTime(32);	// must wait a while see page 51 of datasheet
    	
    	float f = bme.readPressure();
    	payload.pressure = f * 100;
		f = bme.readTemperature();
		payload.temp = f * 100;
		f = bme.readHumidity();
		payload.humi = f * 100;
		bme.write8(0xE0, 0xB6);	// Soft reset this makes sure the IIR is off, etc.
    	Sleepy::loseSomeTime(32);	// Allow power to settle
	#elif BMP280_PORT
    	bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
			Adafruit_BMP280::SAMPLING_X1, // temperature
            Adafruit_BMP280::SAMPLING_X1, // pressure
            Adafruit_BMP280::FILTER_OFF   );
    	// Only needed in forced mode! In normal mode, you can remove the next line.
//    	bmp.takeForcedMeasurement();// has no effect in normal mode
    	Sleepy::loseSomeTime(32);	// must wait a while see page 51 of datasheet
    	
    	float f = bmp.readPressure();
    	payload.pressure = f * 100;
		f = bmp.readTemperature();
		payload.temp = f * 100;
		bmp.reset();	// Soft reset this makes sure the IIR is off, etc.
    	Sleepy::loseSomeTime(32);	// Allow power to settle
	#endif

	   
    #if LDR_PORT
        ldr.digiWrite2(1);  // enable AIO pull-up
        byte light = ~ ldr.anaRead() >> 2;
        ldr.digiWrite2(0);  // disable pull-up to reduce current draw
        payload.light = smoothedAverage(payload.light, light, firstTime);    
    	firstTime = false;
    #endif
    	
    	if (payload.humi != lastHumi) changed = true;
    	if (payload.temp != lastTemp) changed = true; 
    	if (payload.light != lastLight) changed = true; 
	    lastLight = payload.light;
	    lastTemp = payload.temp;
	    lastHumi = payload.humi;
    #if PIR_PORT
//        payload.moved = 0;//pir.state();
    #endif
#if SERIAL
	#if BME280_PORT
        Serial.print("ROOM_BME280 ");
	#elif BMP280_PORT
        Serial.print("ROOM_BMP280 ");
    #endif
        Serial.print((int) payload.light);
        Serial.print(' ');
//        Serial.print((int) payload.moved);
        Serial.print(" h=");
        float x = payload.humi / 100.0f;
        Serial.print(x);
        Serial.print("% t=");
        x = payload.temp / 100.0f;
        Serial.print(x);
        Serial.print("°C ");
//        Serial.print((int) payload.lobat);
//        Serial.print(' ');
//        Serial.print((int) countPCINT);
//        Serial.print(" p=");
        Serial.print("p=");
        x = payload.pressure / 100.0f;
        Serial.print(x);
        Serial.print("hPa Vcc=");
        Serial.println(payload.vcc);
        serialFlush();
#endif
		if (++measureCount >= settings.REPORT_EVERY) {
			scheduler.timer(REPORT, 1);
			measureCount = 0;
		}

} // doMeasure

#if SERIAL
static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2l); // make sure tx buf is empty before going back to sleep
}
#endif

// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
    payload.attempts = 0;
    rf12_sleep(RF12_WAKEUP);
    rf12_sendNow(0, &payload, payloadLength);
    rf12_sendWait(RADIO_SYNC_MODE);
    rf12_sleep(RF12_SLEEP);
}

// send packet and wait for ack when there is a motion trigger
static void doTrigger() {
	bool releaseAck = false;

    if (rf12_recvDone()) {
//		showString(PSTR("Discarded: "));	// Flush the buffer
        for (byte i = 0; i < 8; i++) {
//            showByte(rf12_buf[i]);
            rf12_buf[i] = 0xFF;			// Paint it over
//            printOneChar(' ');
        }
//		Serial.println();
	}

	payload.sequence++;
#if SERIAL
	Serial.print("Sequence ");
	Serial.println(payload.sequence);
#endif
    for (byte i = 1; i <= RETRY_LIMIT; ++i) {
    	payload.attempts = i;
        rf12_sleep(RF12_WAKEUP);
/*
    	while (!(rf12_canSend())) {
    #if SERIAL
			showString(PSTR("Airwaves Busy\n")); serialFlush();
	#endif
    	Sleepy::loseSomeTime(32);	// Wait a while
		}
*/		
	#if SERIAL
    	Serial.print("Transmitting ");
		Serial.print(payloadLength);
		showString(PSTR(" @ "));
		Serial.print(rfapi.txPower);
		showString(PSTR(" ackPacer "));
		Serial.println(ackPacer); serialFlush();
	#endif
		if ( !(ackPacer + settings.ackBounds) ) ackPacer = 1;
		if (ackPacer-- <= 0) {
			ackSW = 0;
			if (settings.ackBounds + ackPacer) payload.command = settings.ackBounds + ackPacer;
    		else payload.command = 85;							// Countdown to next Ack request
		}
		else 
		{
			ackSW = RF12_HDR_ACK;
		} 
		clock_prescale(RADIOSPEED);
		rf12_sendStart(ackSW, &payload, payloadLength);
        rf12_sendWait(RADIO_SYNC_MODE);	// Don't slow processor for this :-(
        
/*		
		for (byte tick = 0; tick < 11; tick++) NOP;	// Kill some time
*/
		if (ackSW)
		{
//			clock_prescale(RADIOSPEED);
        	byte acked = waitForAck();
 			clock_prescale(IDLESPEED);
   	
        	if (acked)
        	{
        		if (rebootRequested) asm volatile ("  jmp 0");  
				clock_prescale(IDLESPEED);
#if RF69_COMPAT
	#if SERIAL
				Serial.print(" Inbound packet at ");
				Serial.print(payload.inboundRssi);
				Serial.print(" with threshold of ");
				Serial.print(rfapi.rssiThreshold);
				Serial.print(", setting new threshold to ");
				Serial.println( (payload.inboundRssi + 3) );
	#endif
				rfapi.rssiThreshold = (payload.inboundRssi + 3);
#else
	#if SERIAL
				Serial.print("Threshold was ");
				Serial.print(rfapi.rssiThreshold);			
				Serial.print(" LNA was "); 
				Serial.println(rfapi.lna);
	#endif
				if ( (rfapi.rssiThreshold & 7) < 5) rfapi.rssiThreshold++;
				else									// Reduce LNA
				if ( (rfapi.rssiThreshold >> 3) < 3) rfapi.rssiThreshold+=3;			
#endif
				payload.rssiThreshold = rfapi.rssiThreshold;
				payloadLength = BASIC_PAYLOADLENGTH;			// Reset to typical
				if (rf12_buf[2] == 1)
				{
					payload.powerSeenAs = rf12_buf[3];
#if SERIAL
					showString(PSTR("Central saw my last packet at power ")); 
					Serial.println(rf12_buf[3]);
#endif
					payload.command = 85;// Clear alert after a node restart
								
          			if (payload.vcc < settings.lowVcc) payload.command = 240;
#if RF69_COMPAT          		
          			if ( (payload.powerSeenAs > 180) && (rfapi.txPower < 159) ) rfapi.txPower++;
          			else
          			if ( (payload.powerSeenAs < 180) && (rfapi.txPower > 128) ) rfapi.txPower--;
#else										// RFM12B: smaller value are more powerful TX
          			if ( (payload.powerSeenAs < 180) && (rfapi.txPower < 7) ) rfapi.txPower++;
          			else
          			if ( (payload.powerSeenAs > 180) && (rfapi.txPower > 0) ) rfapi.txPower--;
#endif
          			payload.sendingPower = rfapi.txPower;
          			break;
				} // if (rf12_buf[2] == 1) 
				else
				if ( (rf12_buf[2] > 1) && (rf12_buf[2] <= 4) )
				{
					ackPacer++;
	            	payload.command = rf12_buf[3];		// Acknowledge the command
	            	releaseAck = true;
					uint16_t value = 0;
					if (rf12_buf[2] == 4) 
						value = ( (rf12_buf[6] << 8) + rf12_buf[5] );
#if SERIAL
						Serial.print("Key:");
						Serial.print(rf12_buf[3]);
						Serial.print(", Flag:");
						Serial.print(rf12_buf[4]);
						Serial.print(", Value1:");
						Serial.print(rf12_buf[5]);
						Serial.print(", Value2:");
						Serial.print(rf12_buf[6]);
						Serial.print(", Value=");
						Serial.println(value);
#endif
					switch (rf12_buf[4]) {
//						case 0				// Flags 0 through to 15 are
//						case 15				// reserved for error codes
						case 20:
							settings.changedLight = false;
                      		break;      
						case 21:
							settings.changedLight = true;
                      		break;      
						case 30:
							settings.changedHumi = false;
                      		break;      
						case 31:
							settings.changedHumi = true;
                      		break;      
						case 40:
						settings.changedTemp = false;
                      	break;      
						case 41:
							settings.changedLight = true;
                      		break; 
						case 50:
							if(rf12_buf[2] == 4) settings.lowVcc = value;
                      		break; 
						case 60:
							ackPacer = 1;
                      	break; 
						case 61:
							if( (rf12_buf[2] == 4) && (value < 128) ) ackPacer = value;
							else ackPacer = 127;
                      	break; 
//                  	case 85 is reserved     
						case 99:
#if SERIAL
							Serial.println("Saving settings to eeprom");
#endif
                      		saveSettings();
                      		break;      
						case 100:
							settings.MEASURE = false;
                  			break;
						case 101:
							settings.MEASURE = true;
							if(rf12_buf[2] == 4) settings.MEASURE_PERIOD = value;
                  			break;
//                  	case 170 is reserved     
//						case 200:
//							settings.REPORT = false;
//                  		break;
						case 201:
							settings.REPORT = true;
							if(rf12_buf[2] == 4) settings.REPORT_EVERY = value;
                  			break;
						case 203:
							settings.MEASURE = true;
							if(rf12_buf[2] == 4) 
								settings.MEASURE_PERIOD = value;
								settings.REPORT_EVERY = 1;
                  			break;
						case 204:
							settings.MEASURE = true;
							if( (rf12_buf[2] == 4) && (value < 128) ) 
								settings.ackBounds = value;
							else payload.command = 170;		// Rejected command	
                  			break;
//                  	case 240 is reserved
						case 254: 		// Reboot
                    		 if (value == 254) rebootRequested = true;
                    		 break;                    
						case 255: 		// Reboot
							if (value == 255) rebootRequested = true;
                    	 	break;
                    	default:                   
#if SERIAL
							Serial.println("Unknown Command");
#endif
	            			payload.command = 170;		// Rejected command									                       
    						return;
//                     		break;
                     	   
                  		} // end switch
          		}
          		else // if ( (rf12_buf[2] > 1)
          		{          	
#if SERIAL
					Serial.print("Unknown ACK type ");
					Serial.println( rf12_buf[2] );
#endif
	            	payload.command = 170;		// Rejected command
	            	releaseAck = true;
	            	return;							                                 	
          		} // if ( (rf12_buf[2] > 1)
        	}
        	else // if (acked)
        	{
        		if (ackPacer < 127) ackPacer++;
	    		payload.missedACK++;

    		} // if (acked)
    	}
    	else // if (ackSW)
    	{
//    		payload.command = settings.ackBounds + ackPacer;	// Countdown to next Ack request
//    		if ( !(payload.command) ) payload.command = 85;	// No Alert on first Acked packet
	    	break;
	    }
	} // RETRY_LIMIT
	
    clock_prescale(IDLESPEED);
    
    if (releaseAck)
		scheduler.timer(REPORT, 1 );
	
} // doTrigger

static byte waitForAck() {

#if SERIAL
//    Serial.print(" Waiting for for ACK ");
#endif
    MilliTimer ackTimer;	// How does this react to clock_prescale
    while ( !(ackTimer.poll(ACK_TIME)) ) {
        if (rf12_recvDone()) {
            rf12_sleep(RF12_SLEEP);
        	byte ack_delay = ( (ACK_TIME) - ackTimer.remaining() );
			payload.inboundRssi = rf12_rssi;
#if SERIAL
			clock_prescale(IDLESPEED);
            Serial.println();
            Serial.print(ack_delay);
            showString(PSTR("ms RX")); serialFlush();
#endif            
            if (rf12_crc == 0) {                          // Valid packet?
                // see http://talk.jeelabs.net/topic/811#post-4712
				if (rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID)) {
					payload.ack_delay = ack_delay;
#if SERIAL
                    showString(PSTR(" ACK "));
                    showByte(payload.attempts);
                    printOneChar(' ');
#endif
                    return true;            
                }  else {
#if SERIAL
                	Serial.print(rf12_hdr, HEX); printOneChar(' ');
					Serial.print((RF12_HDR_DST | RF12_HDR_CTL | myNodeID), HEX);
					showString(PSTR(" Unmatched: "));	// Flush the buffer
#endif
/*
                    for (byte i = 0; i < 8; i++) {
#if SERIAL
                        showByte(rf12_buf[i]);
                        printOneChar(' ');
#endif
                        rf12_buf[i] = 0xFF;				// Paint it over
                    }
*/
                    payload.command = 3;	// Wrong packet
#if SERIAL
                    Serial.println();
#endif
					return false;
                }
            } else { 
            	payload.command = 1;	// CRC bad
            	payload.badCRC++;
#if SERIAL
            	showString(PSTR("Bad CRC"));	            
				Serial.println();serialFlush();
#endif
				return false;
			}          
        }
		payload.inboundRssi = rf12_rssi;	// Whatever we may have heard
#if SERIAL
//		clock_prescale(IDLESPEED);
//		printOneChar('.');serialFlush();
#endif        
    }
    rf12_sleep(RF12_SLEEP);
    payload.command = 2;	// Ack timeout
    payload.lna = rfapi.lna;
    payload.fei = rfapi.fei;
    payloadLength = TIMEOUT_PAYLOADLENGTH;
	payload.ack_delay = 0;
#if SERIAL
	clock_prescale(IDLESPEED);
	Serial.println();
	Serial.print(ACK_TIME);
	showString(PSTR("ms ACK Timeout "));
#endif
#if RF69_COMPAT
	if (rfapi.rssiThreshold < 210) rfapi.rssiThreshold += 5;                    
	if ( (rfapi.txPower < 159) ) rfapi.txPower++;
#else
	if ( (rfapi.rssiThreshold & 7) > 0) rfapi.rssiThreshold--;		// Increase RX threshold
	else
	if ( (rfapi.rssiThreshold >> 3) > 0) rfapi.rssiThreshold-=3;	// LNA			

	if ( (rfapi.txPower > 0) ) rfapi.txPower--;			// Increase TX power
#endif
#if SERIAL
    showString(PSTR(" Increasing threshold to "));
    Serial.print(rfapi.rssiThreshold);
    showString(PSTR(" Increasing transmit power "));
	Serial.println(rfapi.txPower); serialFlush();
#endif
    return false;
} // waitForAck

void blink (byte pin) {
    for (byte i = 0; i < 6; ++i) {
        delay(100l);
        digitalWrite(pin, !digitalRead(pin));
    }
}

static void saveSettings () {
	wdt_reset();
    settings.start = ~0;
    settings.crc = calcCrc(&settings, sizeof settings - 2);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why
    for (byte i = 0; i < sizeof settings; ++i) {
        byte* p = &settings.start;
        payload.message[i] = eeprom_read_byte(SETTINGS_EEPROM_ADDR + i);
        if ((byte)payload.message[i] != (byte)p[i]) {
        	delay(1);	// Avoid brownout?
            eeprom_write_byte(SETTINGS_EEPROM_ADDR + i, p[i]);
#if SERIAL
     		Serial.print("Eeprom change ");
     		Serial.print(i);
     		printOneChar(' ');
     		Serial.print( (byte)payload.message[i] );
     		printOneChar('>');
     		Serial.println( p[i] );
#endif
        }
    }
    payloadLength = EXTENDED_PAYLOADLENGTH + (sizeof settings);
} // saveSettings

static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}

static void loadSettings () {
    uint16_t crc = ~0;
    // eeprom_read_block(&settings, SETTINGS_EEPROM_ADDR, sizeof settings);
    // this uses 166 bytes less flash than eeprom_read_block(), no idea why
    for (byte i = 0; i < sizeof settings; ++i) {
    	payload.message[i] = ((byte*) &settings)[i] 
        	= eeprom_read_byte(SETTINGS_EEPROM_ADDR + i);
        crc = crc_update(crc, ((byte*) &settings)[i]);
    }
    payloadLength = EXTENDED_PAYLOADLENGTH + (sizeof settings);
#if SERIAL
     Serial.print("Settings CRC ");
#endif     
    if (crc) {
#if SERIAL
		Serial.print("is bad, defaulting ");
		Serial.println(crc, HEX);
#endif
        settings.MEASURE_PERIOD = 600;
        settings.REPORT_EVERY = 1;
        settings.MEASURE = settings.REPORT = true;
        settings.lowVcc = 140;
        settings.ackBounds = 60;
        settings.changedLight = settings.changedHumi = settings.changedTemp = true;
    } 
#if SERIAL    
    else {
		Serial.println("is good");
		
		showString(PSTR("EEprom Measure Period "));
		Serial.print(settings.MEASURE_PERIOD);
		showString(PSTR(" Report Every "));
		Serial.println(settings.REPORT_EVERY);
        settings.MEASURE_PERIOD = 60;	// Override eeprom if on serial port
        settings.REPORT_EVERY = 1;
        settings.ackBounds = 30;		
    }
#endif
} // loadSettings

#if SERIAL
static void printOneChar (char c) {
     Serial.print(c);
}

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
     Serial.print(c);
}

static void showByte (byte value) {
//    if (config.output & 0x1) {
        showNibble(value >> 4);
        showNibble(value);
//    } else
//         Serial.print((word) value, DEC);
}

static void showWord (unsigned int value) {
//    if (config.output & 0x1) {
        showByte (value >> 8);
        showByte (value);
//    } else
//         Serial.print((word) value);    
}

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static void dumpRegs() {

	showString(PSTR("\nRadio Registers:\n"));      
	showString(PSTR("    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n"));      
    for (byte i = 0; i < 0x80; i+=16) {
    	showNibble(i >> 4); showNibble(i); printOneChar(':');
        for (byte j=0; j<16; j++)
            if (i==0 && j==0) showString(PSTR(" --")); 
            else {
    			printOneChar(' ');
	            byte r = RF69::control((i + j), 0);
    			showNibble(r >> 4); showNibble(r);
    		}
    		Serial.println();
    }
	
}
#endif

void setup () 
{
//	delay(250);
//disable interrupts
	cli();
// Setup WatchDog
	wdt_reset();			// First thing, turn it off
	MCUSR = 0;
	payload.rebootCode = resetFlags;
	MCUSR = 0;
	wdt_disable();
	wdt_enable(WDTO_8S);	// enable watchdogtimer at 8 seconds
// Enable global interrupts
	sei();

    clock_prescale(IDLESPEED);	// Divide clock by 4, Serial viewable at 2400
#if SERIAL || DEBUG
    Serial.begin(38400);
    Serial.print("[roomNode.3.2] ");
	Serial.print("Reboot Code: 0x");
	Serial.println( payload.rebootCode, HEX );   
    serialFlush();
    myNodeID = rf12_config(true);
#else
	myNodeID = rf12_config(0); // don't report info on the serial port
#endif
	loadSettings();
	
    rf12_sleep(RF12_SLEEP); // power down
#if SERIAL
	Serial.print("Transmit Power "); Serial.println(rfapi.txPower);
	Serial.print(settings.MEASURE_PERIOD);
	Serial.print(" Measure and report every ");
	Serial.println(settings.REPORT_EVERY);
	serialFlush();
#endif    
    
#if BME280_PORT
	if (! bme.begin(BMX280_ADDRESS)) {
#elif BMP280_PORT
	if (! bmp.begin(BMX280_ADDRESS)) {
#endif
#if SERIAL
    	 Serial.println("Could not find a valid BME280 or BMP280 sensor"); serialFlush();
#endif
    	
    }

#if PIR_PORT
	pir.digiWrite(PIR_PULLUP);
	#ifdef PCMSK2
    bitSet(PCMSK2, PIR_PORT + 3);
    bitSet(PCICR, PCIE2);
#else
//XXX TINY!
#endif
#endif

    if (settings.MEASURE)
		scheduler.timer(MEASURE, 10);
//    scheduler.timer(REPORT, 10);
    	
/*
uint16_t lastPass = 0; 
    rf12_sleep(RF12_WAKEUP);
 	dumpRegs();  
 while (1) {
 	if (RF69::RXinterruptCount != lastPass) {
 		lastPass = RF69::RXinterruptCount;
//	Serial.print("I=");
//	Serial.println(RF69::interruptCount);
//	Serial.print("TX=");
//	Serial.println(RF69::TXinterruptCount);
//	Serial.print("RX=");
	Serial.println(RF69::RXinterruptCount);
	}
	if ( rf12_recvDone() ) {
		showString(PSTR("Discarded: "));	// Flush the buffer
        for (byte i = 0; i < 8; i++) {
            showByte(rf12_buf[i]);
            rf12_buf[i] = 0xFF;			// Paint it over
            printOneChar(' ');
        }
		Serial.println();
	}

}
*/ 
} // Setup

void loop () 
{
/*
    #if PIR_PORT
        if (pir.triggered()) {
//            payload.moved = 0;//pir.state();
			clock_prescale(IDLESPEED);
			doTrigger();
        }
    #endif
*/	
	wdt_disable();			// Disable since pollWaiting has an extended delay    
	byte s = scheduler.pollWaiting();
	wdt_enable(WDTO_8S);	// enable watchdogtimer at 8 seconds

	switch (s) {
		case MEASURE:
        // reschedule these measurements periodically
        	if (settings.MEASURE) {
				scheduler.timer(MEASURE, settings.MEASURE_PERIOD);
			}
            clock_prescale(IDLESPEED);
            doMeasure();
            break;
            
        case REPORT:
            clock_prescale(IDLESPEED);
    		#if PIR_PORT
    		maskPCINT = true;	// Airwick PIR is skittish
			#endif
		
//            doReport();
            doTrigger();

	    	#if PIR_PORT
    		maskPCINT = false;
	        #endif
            break;
    } // end switch (s)
#if SERIAL
	clock_prescale(IDLESPEED);
//	Serial.println("Loop");    
	serialFlush();
#endif
	clock_prescale(8);

} // Loop
