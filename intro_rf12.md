RF12 wireless driver
====================

This is a driver for the low-cost "RFM12" and "RFM12B" wireless transceiver
modules from HopeRF. These are available for the 433, 868, and 915 MHz
frequency bands and support a simple but effective byte packet format for
sending and receiving small amounts of "signaling" data.

The driver is written in C and works on ATmega and ATtiny microcontrollers.
Others have ported it to other microncontrollers, such as MSP430 and ARM.

The **RF12demo** sketch can be used as general-purpose send or receive node,
this can be useful while wriitng and testing your own code with this driver.

* Main functions: 
rf12_initialize(),
rf12_recvDone(),
rf12_canSend(),
rf12_sendStart(),
rf12_sendNow()
* Easy transmit wrappers:
rf12_easyInit(),
rf12_easyPoll(),
rf12_easySend()
* Alternate initialisation:
rf12_set_cs(),
rf12_spiInit(),
rf12_config()
* Other functions:
rf12_control(),
rf12_encrypt(),
rf12_lowbat(),
rf12_onOff(),
rf12_sendWait(),
rf12_sleep()

See the RF12.h and RF12.cpp files for the actual source code.

TBD
===

More info coming soon... overview, examples, etc.
