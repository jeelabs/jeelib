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
#include <avr/eeprom.h>
#include <util/crc16.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define crc_update      _crc16_update

#define SERIAL  0   // set to 1 to also report readings on the serial port
#define DEBUG   0   // set to 1 to display each loop() run and PIR trigger

#define BME280_PORT  1   // defined if BME280 is connected to I2C
// #define SHT11_PORT  1   // defined if SHT11 is connected to a port
//	#define HYT131_PORT 1   // defined if HYT131 is connected to a port
#define LDR_PORT    4   // defined if LDR is connected to a port's AIO pin
#define PIR_PORT    4   // defined if PIR is connected to a port's DIO pin

//#define MEASURE_PERIOD  600 // how often to measure, in tenths of seconds
#define RETRY_PERIOD    10  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack
//#define REPORT_EVERY    60  // report every N measurement cycles
#define SMOOTH          3   // smoothing factor used for running averages

#define SETTINGS_EEPROM_ADDR ((uint8_t*) 0x00)

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

// The scheduler makes it easy to perform various tasks at various times:

enum { MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);
#if BME280_PORT    
	Adafruit_BME280 bme; // I2C
#endif

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
    	// Only needed in forced mode! In normal mode, you can remove the next line.
    	bme.takeForcedMeasurement();// has no effect in normal mode
    	Sleepy::loseSomeTime(32);	// must wait a while see page 51 of datasheet
    	payload.pressure = bme.readPressure();
		payload.temp = bme.readTemperature();
		payload.humi = bme.readHumidity();
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
    delay(2); // make sure tx buf is empty before going back to sleep
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
        Serial.print("hPa VCC=");
        Serial.println(payload.vcc);
        serialFlush();
		clock_prescale(8);
    #endif
}

// send packet and wait for ack when there is a motion trigger
static void doTrigger() {
    #if DEBUG
        Serial.print("\nPIR ");
//        Serial.print((int) payload.moved);
        Serial.print(' ');
        Serial.print((int) countPCINT);
        serialFlush();
    #endif
    
//    payload.count++;
    for (byte i = 1; i < RETRY_LIMIT; ++i) {
    	payload.attempts = i;
        rf12_sleep(RF12_WAKEUP);
        rf12_sendNow(RF12_HDR_ACK, &payload, sizeof payload);
        rf12_sendWait(RADIO_SYNC_MODE);
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);

        if (acked) {
        #if DEBUG
            Serial.print(" ack ");
            Serial.println((int) i);
            serialFlush();
        #endif

	        if (rf12_buf[2] > 0) {                  // Non-zero length ACK packet?
	            payload.command = rf12_buf[3];		// Acknowledge the command

				switch (rf12_buf[3]) {
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
					case 99:
						//Serial.println("Saving settings to eeprom");
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
                       // Serial.println("Unknown Command");
                     	break;
                  	} // end switch
          	}
            // reset scheduling to start a fresh measurement cycle
            scheduler.timer(MEASURE, settings.MEASURE_PERIOD);
            return;
        }
        
    	payload.missedACK++;
        delay(RETRY_PERIOD * 100);
    }
    scheduler.timer(MEASURE, settings.MEASURE_PERIOD);
    #if DEBUG
        Serial.println(" no ack!");
        serialFlush();
    #endif
}

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

void blink (byte pin) {
    for (byte i = 0; i < 6; ++i) {
        delay(100);
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
     //Serial.print("Settings CRC ");
    if (crc) {
         //Serial.println("is bad, defaulting");
         //Serial.println(crc, HEX);
        settings.MEASURE_PERIOD = 135;//540;	// approximately 60 seconds
        settings.REPORT_EVERY = 60;		// approximately 60 minutes
        settings.MEASURE = settings.REPORT = true;
        settings.changedLight = settings.changedHumi = settings.changedTemp = true;
    } else {
         //Serial.println("is good");
    }
} // loadSettings

void setup () {
//	payload.spare2 = 3;
    #if SERIAL || DEBUG
        Serial.begin(115200);
        Serial.print("\n[roomNode.3.1]");
        myNodeID = rf12_config();
        serialFlush();
    #else
        myNodeID = rf12_config(0); // don't report info on the serial port
    #endif
    
	loadSettings();
    
    rf12_sleep(RF12_SLEEP); // power down
    
	#if BME280_PORT    
    	if (! bme.begin(0x76)) {
    	#if DEBUG
    		Serial.println("Could not find a valid BME280 sensor");
    	#endif
    		while (1);
    	}
    	bme.setSampling(Adafruit_BME280::MODE_FORCED,
			Adafruit_BME280::SAMPLING_X1, // temperature
            Adafruit_BME280::SAMPLING_X1, // pressure
            Adafruit_BME280::SAMPLING_X1, // humidity
            Adafruit_BME280::FILTER_OFF   );
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

    reportCount = settings.REPORT_EVERY;     // report right away for easy debugging
    scheduler.timer(MEASURE, 0);    // start the measurement loop going
}

void loop () {
    #if DEBUG
        Serial.print('.');
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
	            if ((++reportCount >= settings.REPORT_EVERY) || (changed)){
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
		
            doReport();

    	#if PIR_PORT
    		maskPCINT = false;
        #endif
            break;
    }
}
