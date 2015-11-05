#!/bin/bash
#echo "Start"
####################################################
####################LOCKFILE########################
LOCKDIR="/tmp/JeeCentralMonitor-lock"
if mkdir "${LOCKDIR}" &>/dev/null; then
####################################################
#echo `date "+%d/%m/%Y %X"` $@ >> /etc/heyu/JeeCentralMonitor.txt
yymmdd=`date "+%Y%m%d"`
#echo $yymmdd
	args=("$@")	# Assign to array
#     49 35 145 16 2 0 0 0 0 255 255 0  0    0   0   0   0 
# args 0  1   2  3  4   5 6  7  8   9  10   11  12  13  14  15
#
#     49 35 145 16  1   0 0 52 11 185   16 136  17  34  12 New  (-70dB)
#     62  8   0 49 32 255 0  0  0   0    0   0 244 255   0   0

# Extract 8 bits XX 145 16 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "node = ${args[0]}"	# Get node number
let "node = $node&31" # strip ack/dst/ctl flags
#echo $node #>> /tmp/node
#
# Extract last command received via ACK
let "command = ${args[1]}"
#
# Extract 2 bits of packet type
let "type = ${args[2]} >> 6"
# Extract 6 bits of RX CRC error count
let "badCRC = ${args[2]} & 63" 
#
# Extract 4 bits 62 8 XX 16 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "attempts = ${args[3]} & 15"	# Get attempt number, strip high order 4 bits
#echo ${args[1]} $attempts #>> /tmp/attempts
#
# Extract 4 bits 49 XX 16 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "sequence = ${args[3]} >> 4"	# Get sequence number, loose low order 4 bits
#echo ${args[1]} $sequence #>> /tmp/sequence
#
# Extract battery voltage
let "Voltage = ${args[4]}"
# Extract Salus ID 49 145 XX 2 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "SalusID = ${args[5]}"
#echo $SalusID
# Extract Salus command 49 145 16 X 0 0 0 0 255 255 0 0 0 0 0 0 (-70dB)
let "SalusCommand = ${args[6]}" # 1 = ON & 2 = OFF
#echo $SalusCommand
#
# Extract Salus noise counter
let "SalusNoise = ${args[7]}"
#echo $SalusNoise
# Extract Cold Feed Temperature
let "ColdFeed = (${args[9]} << 8) + ${args[8]}"
# Extract Boiler Feed Temperature
let "BoilerFeed = (${args[11]} << 8) + ${args[10]}"
# Extract Central Heating Return Temperature
let "CHreturn = (${args[13]} << 8) + ${args[12]}"
# Extract Tank Coil Return Temperature
let "TCreturn = (${args[15]} << 8) + ${args[14]}"

# Collect latest temperature from RoomNode
/usr/bin/tail -n1 /etc/heyu/JeeRoomNodeDecoder.dat > /tmp/latestRoomNode.dat # /tmp is a ram disk on RPi
read date time RoomNode light movement humidity temperature  < /tmp/latestRoomNode.dat

# Collect latest temperature from RoomNode
/usr/bin/tail -n1 /etc/heyu/GasMeter > /tmp/GasMeter # /tmp is a ram disk on RPi
read date time GasMeter GasCount  < /tmp/GasMeter

echo `date "+%Y/%m/%d %X"` $node $command $type $badCRC $sequence $attempts $Voltage $SalusID $SalusCommand $SalusNoise $ColdFeed $BoilerFeed $CHreturn $TCreturn $temperature $GasMeter >> /etc/heyu/JeeCentralMonitor.dat

#################################################################
rm -rf "${LOCKDIR}"
else
#echo "Finished"
echo `date "+%Y/%m/%d %X"` $@ >> /etc/heyu/JeeCentralMonitor.skipped
fi
#################################################################
wget -q -O- "https://api.thingspeak.com/update?key=O64IA4X1ZIVU9DWN&field4=$Voltage&field5=$ColdFeed&field6=$BoilerFeed&field7=$CHreturn&field8=$TCreturn" > /dev/null 2>&1
#