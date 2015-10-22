#!/bin/bash
####################################################
####################LOCKFILE########################
LOCKDIR="/tmp/JeeRoomNodeDecoder-lock"
if mkdir "${LOCKDIR}" &>/dev/null; then
####################################################
yymmdd=`date "+%Y%m%d"`
/usr/bin/tail -n1 /etc/heyu/JeeRoomNodeDecoder.dat > /tmp/JeeRoomNodeDecoder.dat               # /tmp is a ram disk on RPi
read date time node light movement humidity temp  < /tmp/JeeRoomNodeDecoder.dat                # Read back last data stored
if [ "${yymmdd:6:2}" -ne "${date:8:2}" ] 
  then
  mv /etc/heyu/JeeRoomNodeDecoder.dat /etc/heyu/archive/JeeRoomNodeDecoder-${date:0:4}${date:5:2}${date:8:2}.dat
  touch /etc/heyu/JeeRoomNodeDecoder.dat
  mv /etc/heyu/JeeRoomNodeDecoder.skipped /etc/heyu/archive/JeeRoomNodeDecoder-${date:0:4}${date:5:2}${date:8:2}.skipped
  touch /etc/heyu/JeeRoomNodeDecoder.skipped
fi
#echo `date "+%Y/%m/%d %X"` $@ >> ./JeeRoomNodeDecoder.txt
	args=("$@")	# Assign to array
#
#     RoomNode
#      4 17 111 12 1 0
# args 0  1   2  3 4 5
#
#struct {
#    byte light;     // (arg 1) light sensor: 0..255
#    byte moved :1;  // (arg 2) motion detector: 0..1
#    byte humi  :7;  // (arg 2) humidity: 0..100
#    int temp   :10; // (arg 3)+(arg 4) temperature: -500..+500 (tenths)
#    byte lobat :1;  // (arg 4) supply voltage dropped under 3.1V: 0..1
#//    int voltage :13; //(arg 4)+(arg 5) AA-Board Battery voltage
#} payload;

let "node = ${args[0]}"	# Get node number
#let "node = $node & 31" # strip ack/dst/ctl flags
let "light = ${args[1]}"	# Get light level
#
# let "timer = 4294967295" 	# maximum timer expected 255 255 255 255
let "movement = ${args[2]} & 1"       # Low order bit indicates movement
#let "movement = $movement / 128"     # Low order bit indicates movement
#
let "humidity = ${args[2]} >> 1"      # 
#let "humidity = $humidity & 127"     # Loose low order bit if present
#
let "temp = ${args[4]} & 3"	        # Clear all but top 2 bits
let "temp = temp << 8"                # Position bits 9&10
let "temp = temp + ${args[3]}"        # Add in bits 1-8
#
echo `date "+%Y/%m/%d %X"` $node $light $movement $humidity $temp >> /etc/heyu/JeeRoomNodeDecoder.dat
#  
# Update COSM
# {
#   "version":"1.0.0",
#   "datastreams":[
#       {"id":"Sequence", "current_value":"2"},
#       {"id":"temperature", "current_value":"4"},
#       {"id":"voltage", "current_value":"4351"}
#   ]
# }
# ,{"id":"Movement", "current_value":"'$movement'"}   
echo '{ "version":"1.0.0","datastreams":[ {"id":"Temperature", "current_value":"'$temp'"},{"id":"Humidity", "current_value":"'$humidity'"},{"id":"Light", "current_value":"'$light'"}]}' > /tmp/RoomNode-cosm.json
/usr/bin/curl --request PUT --data-binary @/tmp/RoomNode-cosm.json --header "X-ApiKey: yBP8ZI0e22YVNnLNIWa2YVlMDdSSAKxSUHZXZWJ6Tm1TND0g" http://api.cosm.com/v2/feeds/84609 >/dev/null 2>&1

else
 echo `date "+%Y/%m/%d %X"` $@ >> /etc/heyu/JeeRoomNodeDecoder.skipped
#################################################################
rm -rf "${LOCKDIR}"
fi
#################################################################

