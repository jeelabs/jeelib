// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#ifndef Ports_h
#define Ports_h

/// @file
/// Ports library definitions.

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <WProgram.h> // Arduino 0022
#endif
#include <stdint.h>
#include <avr/pgmspace.h>
//#include <util/delay.h>

// tweak this to switch ATtiny84 etc to new Arduino 1.0+ conventions
// see http://arduino.cc/forum/index.php/topic,51984.msg371307.html#msg371307
// and http://forum.jeelabs.net/node/1567
#if ARDUINO >= 100
#define WRITE_RESULT size_t
#else
#define WRITE_RESULT void
#endif

/// Interface for JeeNode Ports - see the wiki docs for
/// [JeeNodes](http://jeelabs.net/projects/hardware/wiki/JeeNode) and
/// [pinouts](http://jeelabs.net/projects/hardware/wiki/Pinouts).
/// The Ports class is a thin wrapper around the Arduino's digitalRead(),
/// digitalWrite(), analogRead(), etc. functions. It was designed to simplify
/// the use of the four standard port headers on JeeNodes.
class Port {
protected:
	/// The port number is a small integer mathing the hardware port used.
    /// Port 0 is special, it designates the I2C hardware pins on a JeeNode.
    uint8_t portNum;

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
	/// @return Arduino digital pin number of a Port's D pin (uint8_t).
    inline uint8_t digiPin() const
        { return 0; }
	/// @return Arduino digital pin number of a Port's A pin (uint8_t).
    inline uint8_t digiPin2() const
        { return 2; }
	/// @return Arduino digital pin number of the I pin on all Ports (uint8_t).
    static uint8_t digiPin3()
        { return 1; }
    /// @return Arduino analog pin number of a Port's A pin (uint8_t).
    inline uint8_t anaPin() const
        { return 0; }
#elif defined(__AVR_ATtiny84__)
	/// @return Arduino digital pin number of a Port's D pin (uint8_t).
    inline uint8_t digiPin() const
        { return 12 - 2 * portNum; }
	/// @return Arduino digital pin number of a Port's A pin (uint8_t).
    inline uint8_t digiPin2() const
        { return 11 - 2 * portNum; }
	/// @return Arduino digital pin number of the I pin on all Ports (uint8_t).
    static uint8_t digiPin3()
        { return 3; }
    /// @return Arduino analog pin number of a Port's A pin (uint8_t).
    inline uint8_t anaPin() const
        { return 11 - 2 * portNum; }
#else
	/// @return Arduino digital pin number of a Port's D pin (uint8_t).
    inline uint8_t digiPin() const
        { return portNum ? portNum + 3 : 18; }
	/// @return Arduino digital pin number of a Port's A pin (uint8_t).
    inline uint8_t digiPin2() const
        { return portNum ? portNum + 13 : 19; }
	/// @return Arduino digital pin number of the I pin on all Ports (uint8_t).
    static uint8_t digiPin3()
        { return 3; }
    /// @return Arduino analog pin number of a Port's A pin (uint8_t).
    inline uint8_t anaPin() const
        { return portNum - 1; }
#endif

public:
	///Contructor for a Port.
    inline Port (uint8_t num) : portNum (num) {}

    // DIO pin

    /// Set the pin mode of a Port's D pin. The mode() function member sets the
    /// I/O data direction of the DIO pin associated with a specific port.
    /// @param value INPUT or OUTPUT.
    inline void mode(uint8_t value) const
        { pinMode(digiPin(), value); }
    /// Reads the value of a Port's D pin.
    /// @return High or Low.
    inline uint8_t digiRead() const
        { return digitalRead(digiPin()); }
	/// Write High or Low to a Port's D pin.
    /// @param value High or Low.
    inline void digiWrite(uint8_t value) const
        { return digitalWrite(digiPin(), value); }
    /// Writes a PWM value to a Port's D pin.
    inline void anaWrite(uint8_t val) const
        { analogWrite(digiPin(), val); }
    /// Applies the Arduino pulseIn() function on a Port's D pin.
    inline uint32_t pulse(uint8_t state, uint32_t timeout =1000000L) const
        { return pulseIn(digiPin(), state, timeout); }
    
    // AIO pin

    /// Set the pin mode of a Port's A pin. The mode2() function member sets
    /// the I/O data direction of the AIO pin associated with a specific port.
    /// @param value INPUT or OUTPUT.
    inline void mode2(uint8_t value) const
        { pinMode(digiPin2(), value); }
    /// Reads an analog value from a Port's A pin.
    /// @return int [0..1023]
    inline uint16_t anaRead() const
        { return analogRead(anaPin()); }        
	/// Reads the value of a Port's A pin.
    /// @return High or Low.
    inline uint8_t digiRead2() const
        { return digitalRead(digiPin2()); }
    /// Write High or Low to a Port's A pin.
    /// @param value High or Low.
    inline void digiWrite2(uint8_t value) const
        { return digitalWrite(digiPin2(), value); }
	/// Applies the Arduino pulseIn() function on a Port's A pin.
    /// @see http://arduino.cc/en/Reference/pulseIn for more details.
    inline uint32_t pulse2(uint8_t state, uint32_t timeout =1000000L) const
        { return pulseIn(digiPin2(), state, timeout); }
        
    // IRQ pin (INT1, shared across all ports)

    /// Set the pin mode of the I pin on all Ports. The mode3() function member
    /// sets the I/O direction of the IRQ pin associated with a specific port.
    /// Note that this is the same pin on all ports.
    /// @param value INPUT or OUTPUT.
    static void mode3(uint8_t value)
        { pinMode(digiPin3(), value); }
    /// Reads the value of the I pin on all Ports.
    /// @return High or Low.
    static uint8_t digiRead3()
        { return digitalRead(digiPin3()); }
    /// Writes the value of the I pin on all Ports.
    /// @param value High or Low.
    static void digiWrite3(uint8_t value)
        { return digitalWrite(digiPin3(), value); }
    /// Writes a PWM value to the I pin of all Ports.
    static void anaWrite3(uint8_t val)
        { analogWrite(digiPin3(), val); }
    
    // both pins: data on DIO, clock on AIO

    /// Does Arduino shiftOut() with data on D and clock on A pin of the Port.
    inline void shift(uint8_t bitOrder, uint8_t value) const
        { shiftOut(digiPin(), digiPin2(), bitOrder, value); }
    uint16_t shiftRead(uint8_t bitOrder, uint8_t count =8) const;
    void shiftWrite(uint8_t bitOrder, uint16_t value, uint8_t count =8) const;
};

/// These objects represent remote nodes connected via wireless.
/// Requires the RemotePort and RemoteHandler classes.
class RemoteNode {
public: 
    /// @struct Data
    /// %Data structure exchanged to implement RemoteNode functionality.
    typedef struct {
        uint8_t flags, modes, digiIO, anaOut[2];
        uint16_t anaIn[4]; // only bits 0..11 used
    } Data;

    RemoteNode (char id, uint8_t band, uint8_t group =0);
    
    void poll(uint16_t msecs);

    friend class RemoteHandler;
    friend class RemotePort;
private:
    uint8_t nid;
    uint32_t lastPoll;
    Data data;
};

/// A remote handler is able to deal with information from remote nodes.
class RemoteHandler {
public:
    static void setup(uint8_t id, uint8_t band, uint8_t group =0);
    static uint8_t poll(RemoteNode& node, uint8_t send);
};

/// A remote port is like a local port, bot connected to a remote node.
class RemotePort : protected Port {
    RemoteNode& node;

    inline uint8_t pinBit() const
        { return portNum - 1; }
    inline uint8_t pinBit2() const
        { return portNum + 3; }
public:
    RemotePort (RemoteNode& remote, uint8_t num) : Port (num), node (remote) {}

    void mode(uint8_t value) const;
    uint8_t digiRead() const;
    void digiWrite(uint8_t value) const;
    void anaWrite(uint8_t val) const;
    
    void mode2(uint8_t value) const;    
    uint16_t anaRead() const;
    uint8_t digiRead2() const;
    void digiWrite2(uint8_t value) const;    
};

/// Can be used to drive a software (bit-banged) I2C bus via a Port interface.
/// @todo Speed up the I2C bit I/O, it's far too slow right now.
class PortI2C : public Port {
    uint8_t uswait;
#if 0
// speed test with fast hard-coded version for Port 1:
    inline void hold() const
        { _delay_us(1); }
    inline void sdaOut(uint8_t value) const
        { bitWrite(DDRD, 4, !value); bitWrite(PORTD, 4, value); }
    inline uint8_t sdaIn() const
        { return bitRead(PORTD, 4); }
    inline void sclHi() const
        { hold(); bitWrite(PORTC, 0, 1); }
    inline void sclLo() const
        { hold(); bitWrite(PORTC, 0, 0); }
public:
    enum { KHZMAX, KHZ400, KHZ100, KHZ_SLOW };
#else
    inline void hold() const
        { delayMicroseconds(uswait); }
    inline void sdaOut(uint8_t value) const
        { mode(!value); digiWrite(value); }
    inline uint8_t sdaIn() const
        { return digiRead(); }
    inline void sclHi() const
        { hold(); digiWrite2(1); }
    inline void sclLo() const
        { hold(); digiWrite2(0); }
public:
    enum { KHZMAX = 1, KHZ400 = 2, KHZ100 = 9 };
#endif
    
    /// Creates an instance of class PortI2C
    /// @param num port number corresponding to physical JeeNode port number.
    /// @param rate in microseconds - time delay between bits? (not quite!)
    PortI2C (uint8_t num, uint8_t rate =KHZMAX);
    
    /// Initalize I2C communication on a JeeNode port.
    /// @param addr I2C address of device with which to communicate
    /// @returns 1 if communication succeeded, 0 otherwise
    uint8_t start(uint8_t addr) const;
    /// Terminate transmission on an I2C connection.
    void stop() const;
    /// Send one byte of data to the currently address I2C device.
    /// @param data the data byte to send out
    /// @returns 1 if device acknowledged write, 0 if device did not respond
    uint8_t write(uint8_t data) const;
    /// Read a byte using I2C protocol on a JeeNode port.
    /// @param last pass 1 to signal the last byte read in this bus transaction
    /// @returns data (byte) read from the I2C device
    uint8_t read(uint8_t last) const;
};

/// Each device on the I2C bus needs to be defined using a DeviceI2C instance.
class DeviceI2C {
    const PortI2C& port;
    uint8_t addr;
    
public:
    DeviceI2C(const PortI2C& p, uint8_t me) : port (p), addr (me << 1) {}
    
    /// see if a device answers at an I2C address
    bool isPresent() const;
    
    /// Create a start condition on the I2C bus, and set things up for sending
    /// data to this device.
    /// @returns true if acknowledged by the slave device.
    uint8_t send() const
        { return port.start(addr); }
    /// Create a start condition on the I2C bus, and set things up for receiving
    /// data from this device.
    /// @returns true if acknowledged.
    uint8_t receive() const
        { return port.start(addr | 1); }
    /// Create a stop condition on the I2C bus, ending the current transfer.
    void stop() const
        { port.stop(); }
    /// Write a byte to the currently addressed device. Must be preceded by a
    /// proper PortI2C start() call.
    /// @param data Data byte to be sent.
    /// @returns true if the device acknowledged the byte (accepts more data).
    uint8_t write(uint8_t data) const
        { return port.write(data); }
    /// Read a byte from the currently addressed device. Must be preceded by a
    /// proper PortI2C start() call.
    /// @param last Indicates whether this is the last byte to read. Used to
    ///             respond to the write with a positive or negative ack. 
    ///             Pass 1 if reading the last byte, otherwise pass 0.
    uint8_t read(uint8_t last) const
        { return port.read(last); }
        
    void setAddress(uint8_t me)
        { addr = me << 1; }
};

/// The millisecond timer can be used for timeouts up to 60000 milliseconds.
/// Setting the timeout to zero disables the timer.
///
/// * for periodic use, poll the timer object with "if (timer.poll(123)) ..."
/// * for one-shot use, call "timer.set(123)" and poll as "if (timer.poll())"

class MilliTimer {
    word next;
    byte armed;
public:
    MilliTimer () : armed (0) {}
    
    /// poll until the timer fires
    /// @param ms Periodic repeat rate of the time, omit for a one-shot timer.
    byte poll(word ms =0);
    /// Return the number of milliseconds before the timer will fire
    word remaining() const;
    /// Returns true if the timer is not armed
    byte idle() const { return !armed; }
    /// set the one-shot timeout value
    /// @param ms Timeout value. Timer stops once the timer has fired.
    void set(word ms);
};

/// Low-power utility code using the Watchdog Timer (WDT). Requires a WDT
/// interrupt handler, e.g. EMPTY_INTERRUPT(WDT_vect);
class Sleepy {
public:
    /// start the watchdog timer (or disable it if mode < 0)
    /// @param mode Enable watchdog trigger after "16 << mode" milliseconds 
    ///             (mode 0..9), or disable it (mode < 0).
    /// @note If you use this function, you MUST included a definition of a WDT
    /// interrupt handler in your code. The simplest is to include this line:
    ///
    ///     ISR(WDT_vect) { Sleepy::watchdogEvent(); }
    ///
    /// This will get called when the watchdog fires.
    static void watchdogInterrupts (char mode);
    
    /// enter low-power mode, wake up with watchdog, INT0/1, or pin-change
    static void powerDown ();

    /// flushes pending data in Serial and then enter low-power mode, wake up
    /// with watchdog, INT0/1, or pin-change
    static void flushAndPowerDown ();
    
    /// Spend some time in low-power mode, the timing is only approximate.
    /// @param msecs Number of milliseconds to sleep, in range 0..65535.
    /// @returns 1 if all went normally, or 0 if some other interrupt occurred
    /// @note If you use this function, you MUST included a definition of a WDT
    /// interrupt handler in your code. The simplest is to include this line:
    ///
    ///     ISR(WDT_vect) { Sleepy::watchdogEvent(); }
    ///
    /// This will get called when the watchdog fires.
    static byte loseSomeTime (word msecs);

    /// This must be called from your watchdog interrupt code.
    static void watchdogEvent();
};

/// simple task scheduler for times up to 6000 seconds
class Scheduler {
    word* tasks;
    word remaining;
    byte maxTasks;
    MilliTimer ms100;
public:
    /// initialize for a specified maximum number of tasks
    Scheduler (byte max);
    Scheduler (word* buf, byte max);

    /// Return next task to run, -1 if there are none ready to run, but there
    /// are tasks waiting, or -2 if there are no tasks waiting (i.e. all idle)
    char poll();
    /// same as poll, but wait for event in power-down mode.
    /// Uses Sleepy::loseSomeTime() - see comments there re requiring the
    /// watchdog timer. 
    char pollWaiting();
    
    /// set a task timer, in tenths of seconds
    void timer(byte task, word tenths);
    /// cancel a task timer
    void cancel(byte task);
    
    /// return true if a task timer is not running
    byte idle(byte task) { return tasks[task] == ~0U; }
};

/// Interface for the Blink Plug - see http://jeelabs.org/bp
class BlinkPlug : public Port {
    MilliTimer debounce;
    byte leds, lastState, checkFlags;
public:
	/// Enum containing shorthands for BlinkPlug button states.
    enum { ALL_OFF, ON1, OFF1, ON2, OFF2, SOME_ON, ALL_ON }; // for buttonCheck

    /// Constructor for the BlinkPlug class.
    /// @param port Portnumber the blinkplug is connected to.
    BlinkPlug (byte port)
        : Port (port), leds (0), lastState (0), checkFlags (0) {}
    
    void ledOn(byte mask);
    void ledOff(byte mask);
    /// @return One byte containing the state of both leds.
    byte ledState() const { return leds; }
    
    byte state();
    byte pushed(); // deprecated, don't use in combination with buttonCheck
    byte buttonCheck();
};

/// Interface for the Memory Plug - see http://jeelabs.org/mp
class MemoryPlug : public DeviceI2C {
    uint32_t nextSave;
public:
    MemoryPlug (PortI2C& port)
        : DeviceI2C (port, 0x50), nextSave (0) {}

    void load(word page, byte offset, void* buf, int count);
    void save(word page, byte offset, const void* buf, int count);
};

/// A memory stream can save and reload a stream of bytes on a MemoryPlug.
class MemoryStream {
    MemoryPlug& dev;
    word start, curr;
    char step;
    byte buffer[256], pos;
public:
    MemoryStream (MemoryPlug& plug, word page =0, char dir =1)
            : dev (plug), start (page), curr (page), step (dir), pos (0) {}
    
    long position(byte writing) const;
    byte get();
    void put(byte data);
    word flush();
    void reset();
};

/// Interface for the UART Plug - see http://jeelabs.org/up
class UartPlug : public Print {
    DeviceI2C dev;
    // avoid per-byte access, fill entire buffer instead to reduce I2C overhead
    byte rxbuf[20], in, out;

    void regSet (byte reg, byte value);
    void regRead (byte reg);
    
public:
    UartPlug (PortI2C& port, byte addr)
        : dev (port, addr), in (0), out (0) {}
        
    void begin(long);
    byte available();
    int read();
    void flush();
    virtual WRITE_RESULT write(byte);
};

/// Interface for the Dimmer Plug - see http://jeelabs.org/dp
class DimmerPlug : public DeviceI2C {
public:
    enum {
        MODE1, MODE2,
        PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7,
        PWM8, PWM9, PWM10, PWM11, PWM12, PWM13, PWM14, PWM15,
        GRPPWM, GRPFREQ,
        LEDOUT0, LEDOUT1, LEDOUT2, LEDOUT3,
        SUBADR1, SUBADR2, SUBADR3, ALLCALLADR,
    };

    DimmerPlug (PortI2C& port, byte addr)
        : DeviceI2C (port, addr) {}
    
    void begin ();
    byte getReg(byte reg) const;
    void setReg(byte reg, byte value) const;
    void setMulti(byte reg, ...) const;
};

/// Interface for the Lux Plug - see http://jeelabs.org/xp
class LuxPlug : public DeviceI2C {
    union { byte b[4]; word w[2]; } data;
public:
    enum {
        CONTROL, TIMING,
        THRESHLOWLOW, THRESHLOWHIGH, THRESHHIGHLOW, THRESHHIGHHIGH, INTERRUPT,
        LUXID = 0xA,
        DATA0LOW = 0xC, DATA0HIGH, DATA1LOW, DATA1HIGH,
    };

    LuxPlug (PortI2C& port, byte addr) : DeviceI2C (port, addr) {}

    /// Initialize the LuxPlug. Wait at least 1000 ms after calling this!
    void begin() {
        send();
        write(0xC0 | CONTROL);
        write(3); // power up
        stop();
    }

    ///Power down the lux plug for low power usage.
    void poweroff() {
        send();
        write(0xC0 | CONTROL);
        write(0); // power down
        stop();
    }
    
    void setGain(byte high);
    
    const word* getData();

    word calcLux(byte iGain =0, byte tInt =2) const;
};

// Interface for the HYT131 thermometer/hygrometer - see http://jeelabs.org/2012/06/30/new-hyt131-sensor/
class HYT131 : public DeviceI2C {
public:
    // Constructor for the HYT131 sensor.
    HYT131 (PortI2C& port) : DeviceI2C (port, 0x28) {}
    
    // Execute a reading; results are in tenths of degrees and percent, respectively
    // @param temp in which to store the temperature (int, tenths of degrees C)
    // @param humi in which to store the humidity (int, tenths of percent)
    // @param delayFun (optional) supply delayFun that takes ms delay as argument, for low-power waiting during reading (e.g. Sleepy::loseSomeTime()). By default, delay() is used
    void reading (int& temp, int& humi, byte (*delayFun)(word ms) =0);
};

/// Interface for the Gravity Plug - see http://jeelabs.org/gp
class GravityPlug : public DeviceI2C {
    /// Data storage for getAxes() and sensitivity()
    union { byte b[6]; int w[3]; } data;
public:
    /// Constructor for Gravity Plug.
    GravityPlug (PortI2C& port) : DeviceI2C (port, 0x38) {}

    /// Setup GravityPlug. Call during setup()
    void begin() {}
    /// Set GravityPlug sensitivity.
    /// @param range 2,4,8
    /// @param bw (optional) bandwidth.
    void sensitivity(byte range, word bw =0);

    /// Get accelleration data from GravityPlug.
    /// @return An array with 3 integers. (x,y,z) respectively.
    const int* getAxes();
    /// Read out the temperature (only for BMA150, not the older BMA020)
    /// @return temp, in half deg C steps, from -30C to +50C (i.e. times 2)
    char temperature();
};

/// Interface for the Input Plug - see http://jeelabs.org/ip
class InputPlug : public Port {
    uint8_t slow;
public:
    InputPlug (uint8_t num, uint8_t fix =0) : Port (num), slow (fix) {}
    
    void select(uint8_t channel);
};

/// Interface for the Infrared Plug - see http://jeelabs.org/ir
class InfraredPlug : public Port {
    uint8_t slot, gap, buf [40];
    char fill;
    uint32_t prev;
public:
    /// Initialize with default values for NEC protocol
    InfraredPlug (uint8_t num);
    
    /// Set slot size (us*4) and end-of-data gap (us*256)
    void configure(uint8_t slot4, uint8_t gap256 =80);
    
    /// Call this continuously or at least right after a pin change
    void poll();
    
    /// Returns number of nibbles read, or 0 if not yet ready
    uint8_t done();

    enum { UNKNOWN, NEC, NEC_REP };
    /// Try to decode a received packet, return type of packet
    /// if recognized, the receive buffer will be overwritten with the results
    uint8_t decoder(uint8_t nibbles);
    
    /// Access to the receive buffer
    const uint8_t* buffer() { return buf; }
    
    /// Send out a bit pattern, cycle time is the "slot4" config value
    void send(const uint8_t* data, uint16_t bits);
};

/// Interface for the Heading Board - see http://jeelabs.org/hb
class HeadingBoard : public PortI2C {
    DeviceI2C eeprom, adc, compass;
    Port aux;
    // keep following fields in order:
    word C1, C2, C3, C4, C5, C6, C7;
    byte A, B, C, D, setReset;

    byte eepromByte(byte reg) const;
    void getConstants();
    word adcValue(byte press) const;

public:
    HeadingBoard (int num)
        : PortI2C (num), eeprom (*this, 0x50), adc (*this, 0x77),
          compass (*this, 0x30), aux (5-num), setReset (0x02) {}
    
    void begin();
    void pressure(int& temp, int& pres) const;
    void heading(int& xaxis, int& yaxis);
};

/// Interface for the Modern Device 3-axis Compass board.
/// See http://shop.moderndevice.com/products/3-axis-compass
class CompassBoard : public DeviceI2C {
    int read2 (byte last);
public:
    CompassBoard (PortI2C& port) : DeviceI2C (port, 0x1E) {}

    float heading();
};

/// Interface for the Proximity Plug - see http://jeelabs.org/yp
class ProximityPlug : public DeviceI2C {
public:
    enum {
        FIFO, FAULT, TPSTATUS, TPCONFIG,
        STR1, STR2, STR3, STR4, STR5, STR6, STR7, STR8, 
        ECEMR, MNTPR, MTPR, TASPR, SCR, LPCR, SKTR,
        CONFIG, SINFO,
    };

    ProximityPlug (PortI2C& port, byte num =0)
        : DeviceI2C (port, 0x5C + num) {}
    
    void begin();
    
    void setReg(byte reg, byte value) const;
    byte getReg(byte reg) const;
};

/// Interface for the Analog Plug - see http://jeelabs.org/ap
class AnalogPlug : public DeviceI2C {
  byte config;
public:
  AnalogPlug (const PortI2C& port, byte addr =0x69)
    : DeviceI2C (port, addr), config (0x1C) {}
  
  /// Default mode is channel 1, continuous, 18-bit, gain x1
  void begin (byte mode =0x1C);
  /// Select channel (1..4), must wait to read it out (up to 270 ms for 18-bit)
  void select (byte channel);
  /// Read out 4 bytes, caller will need to shift out the irrelevant lower bits
  long reading ();
};

/// Interface for the DHT11 and DHT22 sensors, does not use floating point
class DHTxx {
  byte pin;
public:
  DHTxx (byte pinNum);
  /// Results are returned in tenths of a degree and percent, respectively.
  /// Set "precise" to true for the more accurate DHT21 and DHT22 sensors.
  bool reading (int& temp, int &humi, bool precise =false);
};

/// Interface for the Color Plug - see http://jeelabs.org/cp
class ColorPlug : public DeviceI2C {
    union { byte b[8]; word w[4]; } data;
    word chromacct[3];
public:
    enum {
        CONTROL, TIMING, INTERRUPT, INTERRUPTSOURCE, CPID, GAIN = 0x7,
        THRESHLOWLOW, THRESHLOWHIGH, THRESHHIGHLOW, THRESHHIGHHIGH,
        DATA0LOW = 0x10, DATA0HIGH, DATA1LOW, DATA1HIGH,
        DATA2LOW, DATA2HIGH, DATA3LOW, DATA3HIGH,
        BLOCKREAD = 0x4F
    };

    ColorPlug (PortI2C& port, byte addr) : DeviceI2C (port, addr) {}
    
    void begin() {
        send();
        write(0x80 | CONTROL);
        write(3); // power up
        stop();
    }
    
    void setGain(byte gain, byte prescaler);
    
    // returns four 16-bit values: red, green, blue, and clear intensities
    const word* getData();
    
    const word* chromaCCT();
};

#ifdef Stream_h // only available in recent Arduino IDE versions

/// Simple parser for input data and one-letter commands
class InputParser {
public:
    typedef struct {
        char code;      // one-letter command code
        byte bytes;     // number of bytes required as input
        void (*fun)();  // code to call for this command
    } Commands;
    
    /// Set up with a buffer of specified size
    InputParser (byte size, Commands*, Stream& =Serial);
    InputParser (byte* buf, byte size, Commands*, Stream& =Serial);
    
    /// Number of data bytes
    byte count() { return fill; }
    
    /// Call this frequently to check for incoming data
    void poll();
    
    InputParser& operator >> (char& v)      { return get(&v, 1); }
    InputParser& operator >> (byte& v)      { return get(&v, 1); }
    InputParser& operator >> (int& v)       { return get(&v, 2); }
    InputParser& operator >> (word& v)      { return get(&v, 2); }
    InputParser& operator >> (long& v)      { return get(&v, 4); }
    InputParser& operator >> (uint32_t& v)  { return get(&v, 4); }
    InputParser& operator >> (const char*& v);

private:
    InputParser& get(void*, byte);
    void reset();
    
    byte *buffer, limit, fill, top, next;
    byte instring, hexmode, hasvalue;
    uint32_t value;
    Commands* cmds;
    Stream& io;
};

#endif // Stream_h

#endif
