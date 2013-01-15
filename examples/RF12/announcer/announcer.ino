/// @dir announcer
/// Test sketch to send out "announcer" type management packets.
/// @see http://jeelabs.org/2013/01/16/remote-node-discovery-code/
// 2013-01-15 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define MINUTES 1   // send out new announcement packets this often, cyclic
#define DEBUG 1     // send quickly, and to serial port instead of wireless

#define SKETCH_roomNode   11
#define SKETCH_ookRelay2  12
#define SKETCH_smaRelay   13
#define SKETCH_otRelay    14
#define SKETCH_p1scanner  15
#define SKETCH_homePower  16
#define SKETCH_radioBlip  17

typedef struct {
  byte nodeId;
  word sketchId;
  byte *items, size;
} NodeInfo;

MilliTimer minuteTimer;
byte nextSend, seqNum;
byte sendBuf[RF12_MAXDATA];

byte items_roomNode_v2[] = {
  0x01-1, 2, // version 2
  0x25-1, 8, 1, 7, -10, 1, // fields are 5 packed bit groups
};

byte items_homePower_v1[] = {
  0x01-1, 1, // version 1
  0x26-1, 16, 16, 16, 16, 16, 16, // fields are 6x 16-bit ints
};

byte items_radioBlip_v1[] = {
  0x01-1, 1, // version 1
  0x21-1, 32, // field is one 32-bit int
};

byte items_otRelay_v1[] = {
  0x01-1, 1, // version 1
  0x22-1, 8, 16, // fields are 2 ints
};

byte items_smaRelay_v1[] = {
  0x01-1, 1, // version 1
  0x27-1, 16, 16, 16, 16, 16, 16, 16, // fields are 7x 16-bit ints
};

#define NUM_NODES (sizeof nodeInfo / sizeof (NodeInfo))

NodeInfo nodeInfo[] = {
  { 2,  SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 3,  SKETCH_radioBlip, items_radioBlip_v1, sizeof items_radioBlip_v1 },
  { 4,  SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 5,  SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 6,  SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 9,  SKETCH_homePower, items_homePower_v1, sizeof items_homePower_v1 },
  { 10, SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 11, SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 12, SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 13, SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 14, SKETCH_otRelay,   items_otRelay_v1,   sizeof items_otRelay_v1   },
  { 15, SKETCH_smaRelay,  items_smaRelay_v1,  sizeof items_smaRelay_v1  },
  { 23, SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
  { 24, SKETCH_roomNode,  items_roomNode_v2,  sizeof items_roomNode_v2  },
};

static void sendAnnouncement (byte index) {
  NodeInfo *nip = &nodeInfo[index];
  // construct header according to the "standard" management packet format
  // see http://jeelabs.org/2013/01/15/remote-node-discovery-part-2/
  sendBuf[0] = (1 << 5) | nip->nodeId;
  sendBuf[1] = ++seqNum;
  sendBuf[2] = nip->sketchId;
  sendBuf[3] = nip->sketchId >> 8;
  // append item data with specific node packet details
  memcpy(sendBuf + 4, nip->items, nip->size);
  byte bytes = 4 + nip->size;

#if !DEBUG
  while (!rf12_canSend())
    rf12_recvDone();
  // special management packet, sent as special "packet to node 0"
  rf12_sendStart(RF12_HDR_DST, sendBuf, bytes);
#endif
  
  Serial.print(index);
  Serial.print(':');
  for (byte i = 0; i < 4 + nip->size; ++i) {
    Serial.print(' ');
    Serial.print(sendBuf[i] >> 4, HEX);
    Serial.print(sendBuf[i] & 0x0F, HEX);
  }
  Serial.println();
}

void setup () {
  Serial.begin(57600);
  Serial.println("\n[announcer]");
#if !DEBUG
  rf12_initialize(1, RF12_868MHZ, 5);
#endif
}

void loop () {
  if (minuteTimer.poll(DEBUG ? 3000 : 60000)) {    
    sendAnnouncement(nextSend);
    if (++nextSend >= NUM_NODES)
      nextSend = 0;
  }
}
