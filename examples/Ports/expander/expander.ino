/// @dir expander
/// Hooking up an 8-bit MCP23008 expander via I2C.
/// @see http://jeelabs.org/2010/02/25/output-and-expander-plug-fixes/
// 2009-07-03 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//
// see http://www.jeelabs.org/2009/07/07/io-expander/

#include <JeeLib.h>

PortI2C myport (1 /*, PortI2C::KHZ400 */);
DeviceI2C expander (myport, 0x20); // also works with output plug if 0x26/0x27

enum {
  MCP_IODIR, MCP_IPOL, MCP_GPINTEN, MCP_DEFVAL, MCP_INTCON, MCP_IOCON,
  MCP_GPPU, MCP_INTF, MCP_INTCAP, MCP_GPIO, MCP_OLAT
};

static void exp_setup () {
    expander.send();
    expander.write(MCP_IODIR);
    expander.write(0); // all outputs
    expander.stop();
}

static void exp_write (byte value) {
    expander.send();
    expander.write(MCP_GPIO);
    expander.write(value);
    expander.stop();
}

// Not used, example code for reading:
//
// static byte exp_read () {
//     expander.send();
//     expander.write(MCP_GPIO);
//     expander.receive();
//     byte result = expander.read(1);
//     expander.stop();
//     return result;
// }

void setup() {
	Serial.begin(57600);
	Serial.println("\n[expander]");
    exp_setup();
}

void loop() {	
	// running light
    for (byte i = 0; i < 8; ++i) {
	    exp_write(1 << i);
        delay(100);
    }
}
