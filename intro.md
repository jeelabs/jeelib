Introduction   {#index}
============

JeeLib is a collection of headers, classes, and sample sketches for use with
the [Arduino IDE][AI]. It was written and extended over the years to simplify
experimenting and exploring the [JeeNode][JN] and other products from
[JeeLabs][JL]. *And what's JeeLabs? Oh, just some guy with a [weblog][WL],
having fun with Physical Computing :)*

The JeeLib code was merged from two separate libraries: "Ports.h" and "RF12.h".
You can still include these headers files, but since you often will need to
use code from both of them, the recommended way to use this library is to add
this single directive at the top of your sketches:

    #include <JeeLib.h>

JeeLib should work with Arduino IDE versions 1.0 and up.

Main contents of this library
-----------------------------

These are the main classes:

* Port - wrappers around pinMode(), digitalRead(), digitalWrite(), etc
* PortI2C - a way to connect to I2C devices on any JeeNode Port
* DeviceI2C - you'll need one of these for each I2C device on the bus
* MilliTimer - a convenient way to track a number of periodic activities
* Sleepy - code to put an ATmega or ATtiny into very low-power mode

The other major implementation in this library is the [RF12 wireless driver]
(md_intro_rf12.html) for the "RFM12" and RFM12B" wireless modules from HopeRF.

Last but not least, there are about a hundred demo sketches in the "examples"
folder, ready to try out all sorts of features and tricks, and to easily hook
up to specific sensors and other interfaces.

Other header files
------------------

There is a BMP085 class with the code to interface to the sensitive pressure +
temperature sensor on the [Pressure Plug][PP]. It's separate from the rest
because the code is larger - to use it, include the "PortsBMP085.h" header.

The LiquidCrustalI2C is derived from LiquidCrystal, which are both defined in
the "PortsLCD.h" header - include it for the [LPCD Plug][LP], for example.

The RF12 class defined in the "RF12sio.h" header is NOT the core %RF12 driver,
but a wrapper to try and send serialized data through via the RFM12B. While it
does work _somewhat_, its use is discouraged. It's too weak for serious use.

Terminology
-----------

In the AVR / ATmega / Arduino / JeeNode world, there are a couple of terms
which are used to designate different things in different contexts. Such as:

* **Pins** - there are pins on an ATmega/ATtiny chip, pins on an Arduino
  board, and pins on a JeeNode port header. Pins all have numbers, but these
  numbers are completely different for each of these contexts.

* **Ports** - an ATmega has hardware ports and a JeeNode has port headers,
  which is a completely different concept. ATmega hardware ports are defined
  in the datasheet from Atmel, and are named A to D. JeeNodes have a number of
  identical 6-pin ports numbered 1-up, to which other hardware can be attached.
  Port 0 is treated specially to use bit-banged I2C on the hardware I2C pins.

* **Plugs** - you might be thinking of connectors, jacks, and sockets, but with
  JeeNodes, "plugs" is the name for little sensor and interface boards which
  connect directly to one of the ports, via their standard 6-pin headers.


[JN]: http://jeelabs.net/projects/hardware/wiki/JeeNode
[JL]: http://jeelabs.com/
[WL]: http://jeelabs.org/
[AI]: http://www.arduino.cc
[LP]: http://jeelabs.net/projects/hardware/wiki/LCD_Plug
[PP]: http://jeelabs.net/projects/hardware/wiki/Pressure_Plug
