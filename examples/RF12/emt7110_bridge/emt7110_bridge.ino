// Bridge data from EMT7110 energy meters to JN RF12 protocol
// (Receiver unit is EMR7370)
// 2014-02-25 <ulf.axelsson@gmail.com> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define EMT7110_PKT_LEN		12
#define EMT7110_DEVICE_TYPE		0x25

#define RAWTRANSMIT

#ifdef RAWTRANSMIT

uint8_t payload[EMT7110_PKT_LEN];

#else

// Packet structure and other information from:
//	http://hobbyelektronik.org/w/index.php/EMR7370
//	http://www.seegel-systeme.de/index.php?page=energiekosten-messgeraet-emr-7330
//	https://www.mail-archive.com/ethersex-devel@list.zerties.org/msg02223.html
//
// This structure and code would normally be used in a receiving JeeNode to interpret the packet
// rather than here since all it does here is to transform the data into LE format (how nice it
// would have been if the AVR would have been BE like its big brother AVR32) 
// 

struct emt7110_data {
	uint8_t device_type;			// Always 0x25 for EMT7110
	uint32_t device_address :24;
	uint8_t pairing_mode :1;		// Is the device in pairing mode?
	uint8_t grid_power :1;			// Is the device connected to the grid?
	uint16_t power :14;			// Power usage in W * 2
	uint16_t current;			// Current usage in mA
	uint8_t voltage;			// Voltage in V * 2 with a 128V offset
	uint8_t unknown1 :1;
	uint8_t unknown2 :1;
	uint16_t energy :14;			// Accumulated Energy consumption in daWh
} payload;

void emt7110_decode(struct emt7110_data *data, const uint8_t* rawdata)
{
	data->device_type = rawdata[0];
	data->device_address = rawdata[1] * 0x10000 + rawdata[2] * 0x100 + rawdata[3];
	data->grid_power = rawdata[4] & 0x40 > 0;
	data->pairing_mode = rawdata[4] & 0x80 > 0;
	data->power = (rawdata[4] & 0x3F) * 0x100 + rawdata[5];
	data->current = rawdata[6] * 0x100 + rawdata[7];
	data->voltage = rawdata[8];
	data->unknown1 = rawdata[9] & 0x80;
	data->unknown2 = rawdata[9] & 0x40;
	data->energy = (rawdata[9] & 0x3F) * 0x100 + rawdata[10];
}

#endif

// Checksum for an EMT7110 packet is calculated by adding all the values including the
// final 12th checksum byte and the result (as a byte) should be 0
uint8_t emt7110_checksum(const uint8_t* raw_data)
{
	uint8_t checksum = 0;
	
	for (int i = 0; i < EMT7110_PKT_LEN; i++)
	checksum += *(raw_data + i);
	
	return checksum;
}

void setup()
{
}

void loop()
{
	rf12_initialize(1, RF12_868MHZ, 0xD4, 0x67C);	// Frequency 868.28 MHz, sync byte 0xD4, node id not relevant
	rf12_control(0xC623);				// Data rate: 9579 baud
	rf12_control(0x9480);				// Bandwidth 180 kHz (as close as we get)
	
	rf12_setRawRecvMode(EMT7110_PKT_LEN);
	while (!rf12_recvDone())
		;
	
	if (rf12_data[0] == EMT7110_DEVICE_TYPE &&
	    emt7110_checksum((const uint8_t*) rf12_data) == 0)
	{

#ifdef RAWTRANSMIT 
		memcpy(&payload, (const void*)rf12_data, EMT7110_PKT_LEN);
#else
		emt7110_decode(&payload, (const uint8_t*)rf12_data);
#endif

		rf12_setRawRecvMode(0);
		rf12_initialize(1, RF12_868MHZ, 5);
		rf12_sendNow(0, &payload, sizeof(payload));
		rf12_sendWait(1);
	}
}
