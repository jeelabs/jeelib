/// @dir announcer
/// Test sketch to send out "announcer" type management packets.
/// @see http://jeelabs.org/2013/01/16/remote-node-discovery-code/
// 2013-01-15 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define DEBUG 0     // send quickly, and to serial port instead of wireless

#define SKETCH_roomNode   11
#define SKETCH_ookRelay2  12
#define SKETCH_smaRelay   13
#define SKETCH_otRelay    14
#define SKETCH_p1scanner  15
#define SKETCH_homePower  16
#define SKETCH_radioBlip  17
#define SKETCH_slowLogger 18

typedef struct {
  byte nodeId;
  word sketchId;
  byte *items, size;
} NodeInfo;

MilliTimer minuteTimer;
byte nextSend, seqNum;
byte sendBuf[RF12_MAXDATA];

#define ITEM(typ,len) (((typ) << 4) | ((len) - 1))

byte items_roomNode_v2[] = {
  ITEM(0,1), 2, // version 2
  ITEM(2,5), 8, 1, 7, -10, 1, // fields are 5 packed bit groups
};

byte items_homePower_v1[] = {
  ITEM(0,1), 1, // version 1
  ITEM(2,6), 16, 16, 16, 16, 16, 16, // fields are 6x 16-bit ints
};

byte items_radioBlip_v1[] = {
  ITEM(0,1), 1, // version 1
  ITEM(2,1), 32, // field is one 32-bit int
};

byte items_otRelay_v1[] = {
  ITEM(0,1), 1, // version 1
  ITEM(2,2), 8, 16, // fields are 2 ints
};

byte items_smaRelay_v1[] = {
  ITEM(0,1), 1, // version 1
  ITEM(2,7), 16, 16, 16, 16, 16, 16, 16, // fields are 7x 16-bit ints
};

byte items_slowLogger_v1[] = {
  ITEM(0,1), 1, // version 1
  ITEM(2,4), 16, 16, 16, 16, // fields are 4x 16-bit ints
};

byte items_v1[] = {
  ITEM(0,1), 1, // generic version 1 item, no other info available
};

#define NUM_NODES (sizeof nodeInfo / sizeof (NodeInfo))

NodeInfo nodeInfo[] = {
  { 2,  SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 3,  SKETCH_radioBlip,  items_radioBlip_v1,  sizeof items_radioBlip_v1  },
  { 4,  SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 5,  SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 6,  SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 9,  SKETCH_homePower,  items_homePower_v1,  sizeof items_homePower_v1  },
  { 10, SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 11, SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 12, SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 13, SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 14, SKETCH_otRelay,    items_otRelay_v1,    sizeof items_otRelay_v1    },
  { 15, SKETCH_smaRelay,   items_smaRelay_v1,   sizeof items_smaRelay_v1   },
  { 18, SKETCH_p1scanner,  items_v1,            sizeof items_v1            },
  { 19, SKETCH_ookRelay2,  items_v1,            sizeof items_v1            },
  { 20, SKETCH_slowLogger, items_slowLogger_v1, sizeof items_slowLogger_v1 },
  { 23, SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
  { 24, SKETCH_roomNode,   items_roomNode_v2,   sizeof items_roomNode_v2   },
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
  // special management packet, sent as special "packet to node 0"
  rf12_sendNow(RF12_HDR_DST, sendBuf, bytes);
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
