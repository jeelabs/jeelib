#!/bin/bash
echo `date "+%d/%m/%Y %X"` $@ >> /etc/heyu/jeebash.txt
#tail -n1 /etc/heyu/jeebash.txt
if [ $# -eq 1 ]	# Arguments received as a single string?
	then
	set -- $@	# Chop it up
fi
	args=("$@")	# Assign to array
#
#     OK 16 5 254 33 0 0 245 10 0 0
# args 0  1 2   3  4 5 6   7  8 9 A
#
#The node number occupies 5 bits.

#The A bit (ACK) 32 indicates whether this packet wants to get an ACK back. The C bit needs to be zero in this case (the name is somewhat confusing).

#The D bit (DST) 64 indicates whether the node ID specifies the destination node or the source node. For packets sent to a specific node, DST = 1. For broadcasts, DST = 0, in which case the node ID refers to the originating node.

#The C bit (CTL) 128 is used to send ACKs, and in turn must be combined with the A bit set to zero.

# Get node number
let node=${args[1]}
let "node=$node&159" # strip dst and ack flags
#echo $@ >> args.txt
#echo $node >> node.txt
#
# If ctl bit is set node will not match below - ignore ACK returns
case $node in

3)  /etc/heyu/JeeRoomNodeDecoder.sh ${args[1]} ${args[2]} ${args[3]} ${args[4]} ${args[5]} ${args[6]} ${args[7]} ${args[8]} ${args[9]} \
                                    ${args[10]} ${args[11]} ${args[12]} ${args[13]} &
                                   echo `date "+%d/%m/%Y %X"` $@ >> /etc/heyu/JeeRoomNodeDecoder.txt                                    
;;
131) 
;;
15) /etc/heyu/JeeGasCounterDecoder.sh ${args[1]} ${args[2]} ${args[3]} ${args[4]} ${args[5]} ${args[6]} ${args[7]} ${args[8]} ${args[9]} &
;;
143) 
;;
17) /etc/heyu/JeeCentralMonitor.sh ${args[1]} ${args[2]} ${args[3]} ${args[4]} ${args[5]} ${args[6]} ${args[7]} ${args[8]} ${args[9]} \
                                   ${args[10]} ${args[11]} ${args[12]} ${args[13]} ${args[14]} ${args[15]} ${args[16]} ${args[17]} \
                                   ${args[18]} ${args[19]} ${args[20]} ${args[21]} ${args[22]} ${args[23]} ${args[24]} ${args[25]} \
                                   ${args[26]} ${args[27]} ${args[28]} ${args[29]} ${args[30]} &
                                   echo `date "+%d/%m/%Y %X"` $@ >> /etc/heyu/JeeCentralMonitor.txt
;;
145) 
;;
*)  echo `date "+%d/%m/%Y %X"` $@ >> /etc/heyu/jeebash.err
;;
esac
