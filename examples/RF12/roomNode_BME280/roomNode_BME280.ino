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

#define RF69_COMPAT      1	 // define this to use the RF69 driver i.s.o. RF12 
///                          // The above flag must be set similarly in RF12.cpp
///                          // and RF69_avr.h

#include <JeeLib.h>
#include <PortsSHT11.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define crc_update      _crc16_update
#define BMX280_ADDRESS	0x76

#define SERIAL  1   // set to 1 to also report readings on the serial port
#define DEBUG   1   // set to 1 to display each loop() run and PIR trigger

#define BME280_PORT  1   // defined if BME280 is connected to I2C
// #define SHT11_PORT  1   // defined if SHT11 is connected to a port
//	#define HYT131_PORT 1   // defined if HYT131 is connected to a port
#define LDR_PORT    4   // defined if LDR is connected to a port's AIO pin
#define PIR_PORT    4   // defined if PIR is connected to a port's DIO pin

//#define MEASURE_PERIOD  600 // how often to measure, in tenths of seconds
#define RETRY_PERIOD    9  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     3   // maximum number of times to retry
#define ACK_TIME        30  // number of milliseconds to wait for an ack
//#define REPORT_EVERY    60  // report every N measurement cycles
#define SMOOTH          3   // smoothing factor used for running averages

#define SETTINGS_EEPROM_ADDR ((uint8_t*) 0x00)

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 0

// The scheduler makes it easy to perform various tasks at various times:

enum { MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);
#if BME280_PORT    
	Adafruit_BME280 bme; // I2C
#endif

static void showString (PGM_P s); // forward declaration

// Other variables used in various places in the code:

static byte reportCount;    // count up until next report, i.e. packet send
static byte myNodeID;       // node ID used for this unit

// This defines the structure of the packets which get sent out by wireless:

struct {					//0		Offset, node #
	byte command;			//1		ACK command return field
    byte missedACK	:4;		//2
    byte attempts	:4;		//2		Transmission attempts
	byte count		:8;		//3		Packet count 
//    byte moved 		:1;  	//3		motion detector: 0..1
//    byte lobat 		:1;  	//3		supply voltage dropped under 3.1V: 0..1
//	byte spare2		:2;		//3    
#if BME280_PORT
    uint32_t pressure:24;	//4&5&6
#endif 	   
    byte light;     		//7		light sensor: 0..255
    unsigned int humi:16;	//8&9	humidity: 0..100.00
    int temp   		:16; 	//10&11	temperature: -5000..+5000 (hundredths)
    byte vcc;				//12	Bandgap battery voltage
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
    word crc;
} eeprom;
static eeprom settings;

int lastTemp;
byte firstTime = true;
byte lastLight;
byte lastHumi;
bool changed;

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
  return (55U * 1023U) / (ADC + 1) - 50;
}

void clock_prescale(uint8_t factor)
{
 if (factor > 8) factor = 8;
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
    #if SERIAL || DEBUG
//	Serial.println("doMeasure"); Serial.flush();
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
    	payload.pressure = bme.readPressure();
		payload.temp = bme.readTemperature();
		payload.humi = bme.readHumidity();

		bme.write8(0xE0, 0xB6);	// Power on reset
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
}

static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2l); // make sure tx buf is empty before going back to sleep
}

// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
    payload.attempts = 0;
    payload.count = payload.count + 1;
    rf12_sleep(RF12_WAKEUP);
    rf12_sendNow(0, &payload, ((sizeof payload) ));
    rf12_sendWait(RADIO_SYNC_MODE);
    rf12_sleep(RF12_SLEEP);

    #if SERIAL
    	clock_prescale(0);
        Serial.print("\nROOM_BME280 ");
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
        Serial.print(' ');
        Serial.print((int) countPCINT);
        Serial.print(" p=");
        x = payload.pressure / 100.0f;
        Serial.print(x);
        Serial.print("hPa Vcc=");
        Serial.println(payload.vcc);
        serialFlush();
    #endif
		clock_prescale(8);
}

// send packet and wait for ack when there is a motion trigger
static void doTrigger() {

    if (rf12_recvDone()) {
		showString(PSTR("Discarded: "));	// Flush the buffer
        for (byte i = 0; i < 8; i++) {
            showByte(rf12_buf[i]);
            rf12_buf[i] = 0xFF;			// Paint it over
            printOneChar(' ');
        }
		Serial.println();
	}
	
    #if DEBUG
//    Serial.print("\nPIR ");
//	Serial.print((int) payload.moved);
//    Serial.print(' ');
//    Serial.println((int) countPCINT);
//    serialFlush();
    #endif
    
//    payload.count++;
    for (byte i = 1; i <= RETRY_LIMIT; ++i) {
//    	delay(4);                   // Increasing the gap between retransmissions
    	payload.attempts = i;
        rf12_sleep(RF12_WAKEUP);
    	while (!(rf12_canSend())) {
    #if SERIAL
			showString(PSTR("Airwaves Busy\n"));
	#endif
			delay(50);
		}
		
    #if SERIAL
//    	Serial.print("Transmitting ");
    #endif
		rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload);
        rf12_sendWait(RADIO_SYNC_MODE);
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);
#if SERIAL
    	Serial.println();
#endif

        if (acked) {

	        if (rf12_buf[2] > 0) {                  // Non-zero length ACK packet?
	            payload.command = rf12_buf[3];		// Acknowledge the command
				reportCount = settings.REPORT_EVERY;// Force a transmission
#if SERIAL
				Serial.print("Key:");
				Serial.println(rf12_buf[3]);
				Serial.print(", Flag:");
				Serial.println(rf12_buf[4]);
				Serial.print(", Value1:");
				Serial.println(rf12_buf[5]);
				Serial.print(", Value2:");
				Serial.println(rf12_buf[6]);
				Serial.print(", Value=");
				uint16_t value = ( (rf12_buf[6] << 8) + rf12_buf[5] );
				Serial.println(value);
#endif
				switch (rf12_buf[4]) {
					case 10:
						settings.changedLight = false;
                      	break;      
					case 11:
						settings.changedLight = true;
                      	break;      
					case 20:
						settings.changedHumi = false;
                      	break;      
					case 21:
						settings.changedHumi = true;
                      	break;      
					case 30:
						settings.changedTemp = false;
                      	break;      
					case 31:
						settings.changedLight = true;
                      	break; 
//                  case 85 is reserved     
					case 99:
#if SERIAL
						Serial.println("Saving settings to eeprom");
#endif
                      	saveSettings();
                      	break;      
					case 100:
						settings.MEASURE = false;
                  		break;
					case 200:
						settings.REPORT = false;
                  		break;

                      	if (rf12_buf[3] > 0 && rf12_buf[3] < 10) {
	                          break;
                      	}
                      	if (rf12_buf[3] > 100 && rf12_buf[3] < 200) {
                      		settings.MEASURE_PERIOD = ((rf12_buf[3]) - 100) * 10;
                      		settings.MEASURE = true;
                          	break;  
                      	}
                      	if (rf12_buf[3] > 200 && rf12_buf[3] <= 255) {
                      		settings.REPORT_EVERY = ((rf12_buf[3]) - 200) * 10;
                      		settings.REPORT = true;
                          	break;
                      	}
#if SERIAL
						Serial.println("Unknown Command");
#endif
	            		payload.command = 170;		// Rejected command									                       
                     	break;
                  	} // end switch
          	} else payload.command = 85;			// Clear alert after a node restart
          	
            // reset scheduling to start a fresh measurement cycle
            scheduler.timer(MEASURE, settings.MEASURE_PERIOD);
            return;
        }
        
    	payload.missedACK++;
        delay(RETRY_PERIOD * 100);
    }

    #if SERIAL
    	clock_prescale(0);
        Serial.print("\nROOM_BME280 Light=");
        Serial.print((int) payload.light);
//        Serial.print(' ');
//        Serial.print((int) payload.moved);
        Serial.print(" h=");
        float x = payload.humi / 100.0f;
        Serial.print(x);
        Serial.print("% t=");
        x = payload.temp / 100.0f;
        Serial.print(x);
        Serial.print("°C ");
//        Serial.print((int) payload.lobat);
        Serial.print("PCINT=");
        Serial.print((int) countPCINT);
        Serial.print(" p=");
        x = payload.pressure / 100.0f;
        Serial.print(x);
        Serial.print("hPa Vcc=");
        Serial.println(payload.vcc);
        serialFlush();
    #endif
    
    scheduler.timer(MEASURE, settings.MEASURE_PERIOD);
    #if DEBUG
        Serial.println(" no ack!");
        serialFlush();
    #endif
		clock_prescale(8);
}

/*
// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}
*/
static byte waitForAck() {
#if SERIAL
//    Serial.print(" Waiting for for ACK ");
#endif
    MilliTimer ackTimer;

    while ( !(ackTimer.poll(ACK_TIME)) ) {
//		Serial.print('W');
        if (rf12_recvDone()) {
            rf12_sleep(RF12_SLEEP);
//		Serial.print('D');

             Serial.print((ACK_TIME) - ackTimer.remaining());
             showString(PSTR("ms RX "));
            
            if (rf12_crc == 0) {                          // Valid packet?
                // see http://talk.jeelabs.net/topic/811#post-4712
				if (rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID)) {
                     showString(PSTR("ACK "));
                    return 1;            
                }  else {
                	Serial.print(rf12_hdr, HEX); printOneChar(' ');
					Serial.print((RF12_HDR_DST | RF12_HDR_CTL | myNodeID), HEX);
					showString(PSTR(" Unmatched: "));             // Flush the buffer
                    for (byte i = 0; i < 8; i++) {
                        showByte(rf12_buf[i]);
                        rf12_buf[i] = 0xFF;              // Paint it over
                        printOneChar(' ');
                    }
                }
            } else {
				showString(PSTR("Bad CRC"));
            }
			Serial.println();Serial.flush();           
        }
//		Serial.print('S');
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
		sleep_mode();
    }
	Serial.println();
	Serial.print(ACK_TIME);
	showString(PSTR("ms ACK Timeout\n"));
	Serial.flush();

    return 0;
} // waitForAck

void blink (byte pin) {
    for (byte i = 0; i < 6; ++i) {
        delay(100l);
        digitalWrite(pin, !digitalRead(pin));
    }
}

static void saveSettings () {
    settings.start = ~0;
    settings.crc = calcCrc(&settings, sizeof settings - 2);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why
    for (byte i = 0; i < sizeof settings; ++i) {
        byte* p = &settings.start;
        if (eeprom_read_byte(SETTINGS_EEPROM_ADDR + i) != p[i]) {
            eeprom_write_byte(SETTINGS_EEPROM_ADDR + i, p[i]);
        }
    }
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
        ((byte*) &settings)[i] = eeprom_read_byte(SETTINGS_EEPROM_ADDR + i);
        crc = crc_update(crc, ((byte*) &settings)[i]);
    }
     Serial.print("Settings CRC ");
     
     
     
     crc = 1;
     
     
     
    if (crc) {
		Serial.println("is bad, defaulting");
		Serial.println(crc, HEX);
        settings.MEASURE_PERIOD = 30;	// approximately 60 seconds
        settings.REPORT_EVERY = 1;		// approximately 1 minute
        settings.MEASURE = settings.REPORT = true;
        settings.changedLight = settings.changedHumi = settings.changedTemp = true;
    } else {
		Serial.println("is good");
    }
} // loadSettings

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

void setup () {
//	payload.spare2 = 3;
    #if SERIAL || DEBUG
        Serial.begin(115200);
        Serial.print("\n[roomNode.3.1]");
        myNodeID = rf12_config(true);
    #else
        myNodeID = rf12_config(0); // don't report info on the serial port
    #endif
	loadSettings();
    rf12_sleep(RF12_SLEEP); // power down
    #if SERIAL
		Serial.print(settings.MEASURE_PERIOD);
		Serial.print(" Measure and report every ");
		Serial.println(settings.REPORT_EVERY);
        serialFlush();
	#endif    
    
	#if BME280_PORT    
    	if (! bme.begin(BMX280_ADDRESS)) {
    	#if SERIAL
    		Serial.println("Could not find a valid BME280 sensor");
    	#endif
    	
    	} else {
    	bme.setSampling(Adafruit_BME280::MODE_FORCED,
			Adafruit_BME280::SAMPLING_X1, // temperature
            Adafruit_BME280::SAMPLING_X1, // pressure
            Adafruit_BME280::SAMPLING_X1, // humidity
            Adafruit_BME280::FILTER_OFF   );
        }
	#endif
    
    #if PIR_PORT
        pir.digiWrite(PIR_PULLUP);
		#ifdef PCMSK2
        bitSet(PCMSK2, PIR_PORT + 3);
        bitSet(PCICR, PCIE2);
		#else
        //XXX TINY!
		#endif
    #endif

    	reportCount = settings.REPORT_EVERY;    // report right away for easy debugging
    	scheduler.timer(MEASURE, 0);    		// start the measurement loop going

/* 
    rf12_sleep(RF12_WAKEUP);
 	dumpRegs();  
 while (1) {
//	Serial.print("I=");
//	Serial.println(RF69::interruptCount);
//	Serial.print("TX=");
//	Serial.println(RF69::TXinterruptCount);
//	Serial.print("RX=");
//	Serial.println(RF69::RXinterruptCount);
	if ( rf12_recvDone() ) {
		showString(PSTR("Discarded: "));	// Flush the buffer
        for (byte i = 0; i < 8; i++) {
            showByte(rf12_buf[i]);
            rf12_buf[i] = 0xFF;			// Paint it over
            printOneChar(' ');
        }
		Serial.println();
	}
	delay (1);
} */   
} // Setup

void loop () {

    #if DEBUG
        serialFlush();
    #endif
    
	clock_prescale(8);

    #if PIR_PORT
        if (pir.triggered()) {
			clock_prescale(0);
//            payload.moved = 0;//pir.state();
			doTrigger();
        }
    #endif
    
    switch (scheduler.pollWaiting()) {

        case MEASURE:
            // reschedule these measurements periodically
            scheduler.timer(MEASURE, settings.MEASURE_PERIOD);
    
            if (settings.MEASURE) {
            	clock_prescale(0);
            	doMeasure();
			}
            // every so often, a report needs to be sent out
            if (settings.REPORT) {
	            if ((++reportCount >= settings.REPORT_EVERY) ){
	            	changed = false;
	                reportCount = 0;
	                scheduler.timer(REPORT, 0);
	            }
            }
            break;
            
        case REPORT:
            clock_prescale(0);
    	#if PIR_PORT
    		maskPCINT = true;	// Airwick PIR is skittish
		#endif
		
//            doReport();
            doTrigger();

    	#if PIR_PORT
    		maskPCINT = false;
        #endif
            break;
    }
} // Loop
