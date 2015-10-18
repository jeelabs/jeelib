#!/bin/bash
#echo "Start"
####################################################
####################LOCKFILE########################
LOCKDIR="/tmp/JeeCentralMonitor-lock"
if mkdir "${LOCKDIR}" &>/dev/null; then
####################################################
echo `date "+%d/%m/%Y %X"` $@ > /tmp/JeeCentralMonitor.txt
yymmdd=`date "+%Y%m%d"`
#echo $yymmdd
	args=("$@")	# Assign to array
#     49 35 145 16 2 0 0 0 0 255 255 0 0 0 0 0 0 
# args 0  1   2  3 4 5 6 7 8   9  101112131415
#
#     49 35 145 16 1 0 0 52 11 185 16 136 17 34 12 (-70dB)

# Extract 8 bits XX 145 16 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "node = ${args[0]}"	# Get node number
let "node = $node&31" # strip ack/dst/ctl flags
#echo $node #>> /tmp/node
#
# Extract last command received via ACK
let "command = ${args[1]}"
#
# Extract 4 bits 49 XX 16 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "attempts = ${args[2]} & 15"	# Get attempt number, strip high order 4 bits
#echo ${args[1]} $attempts #>> /tmp/attempts
#
# Extract 4 bits 49 XX 16 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "sequence = ${args[2]} >> 4"	# Get sequence number, loose low order 4 bits
#echo ${args[1]} $sequence #>> /tmp/sequence
#
# Extract battery voltage
let "Voltage = ${args[3]}"
# Extract Salus ID 49 145 XX 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "SalusID = ${args[4]}"
#echo $SalusID
# Extract Salus command 49 145 16 X 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "SalusCommand = ${args[5]}" # 1 = ON & 2 = OFF
#echo $SalusCommand
#
# Extract Salus noise counter
let "SalusNoise = ${args[6]}"
#echo $SalusNoise
# Extract Cold Feed Temperature
let "ColdFeed = (${args[8]} << 8) + ${args[7]}"
# Extract Boiler Feed Temperature
let "BoilerFeed = (${args[10]} << 8) + ${args[9]}"
# Extract Central Heating Return Temperature
let "CHreturn = (${args[12]} << 8) + ${args[11]}"
# Extract Tank Coil Return Temperature
let "TCreturn = (${args[14]} << 8) + ${args[13]}"

# Collect latest temperature from RoomNode
/usr/bin/tail -n1 /etc/heyu/JeeRoomNodeDecoder.dat > /tmp/latestRoomNode.dat # /tmp is a ram disk on RPi
read date time RoomNode light movement humidity temperature  < /tmp/latestRoomNode.dat
echo `date "+%Y/%m/%d %X"` $node $command $sequence $attempts $Voltage $SalusID $SalusCommand $SalusNoise $ColdFeed $BoilerFeed $CHreturn $TCreturn $temperature >> /etc/heyu/JeeCentralMonitor.dat

#################################################################
rm -rf "${LOCKDIR}"
else
#echo "Finished"
echo `date "+%Y/%m/%d %X"` $@ >> /etc/heyu/JeeCentralMonitor.skipped
fi
#################################################################
wget -q -O- "https://api.thingspeak.com/update?key=O64IA4X1ZIVU9DWN&field4=$Voltage&field5=$ColdFeed&field6=$BoilerFeed&field7=$CHreturn&field8=$TCreturn" > /dev/null 2>&1
#