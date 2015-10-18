#!/bin/bash
if [ $# -ne 9 ]
then
  echo "Incorrect number ($#) of arguments $@" >> /etc/heyu/JeeGasCounterDecoder.err
  exit 65
fi
####################################################
#####             Sandhurst                    #####
####################LOCKFILE########################
LOCKDIR="/tmp/JeeGasCounterDecoder-lock"
if mkdir "${LOCKDIR}" &>/dev/null; then
####################################################
#/usr/local/bin/heyu on Gas_Counter &
yymmdd=`date "+%Y%m%d"`
	args=("$@")	# Assign to array
#     17 42 40 0 65 0 37 150
# args 0  1  2 3  4 5  6   7 
#
let "node = ${args[0]}"	# Get node number
let "sequence = ${args[1]}"	# Get sequence sequence number
#
# let "count = 65535"	    # maximum count expected 255 255
let "count = ${args[3]}"
let "count = $count << 8"
let "count = $count + ${args[2]}"
#
# let "timer = 65535" 	# maximum timer expected 255 255
let "timer = ${args[5]}"
#echo "T1:" $timer >> timer.txt
let "timer = $timer << 8"
#echo "T2:" $timer >> timer.txt
let "timer = $timer + ${args[4]}"
#echo "T3:" $timer >> timer.txt
#
#let "timer = $timer / 1000"	# Reduce to seconds
#echo "T4:" $timer >> timer.txt
#
# let "temperature = 65535" 	# maximum timer expected 255 255
let "temperature = ${args[6]}"
let "temperature = $temperature - 12"    # Calibration
#  
let "voltage = ${args[7]}"
#echo "V1=" $voltage >> voltage.txt
let "voltage = $voltage * 20"
#echo "V2=" $voltage >> voltage.txt
let "voltage = $voltage + 1000"	   
#echo "V3=" $voltage >> voltage.txt
#
/usr/bin/tail -n1 /etc/heyu/GasMeter > /tmp/GasMeter               # /tmp is a ram disk on RPi
read date time meter jeecount  < /tmp/GasMeter           # 1283036400 NNNNNnnn NNN
#echo "DEBUG:($date)($time)($meter)($jeecount)" >> /etc/heyu/debug
#echo ${yymmdd:6:2}  ${date:8:2}
if [ "${yymmdd:6:2}" -ne "${date:8:2}" ] 
  then
  mv /etc/heyu/GasMeter /etc/heyu/archive/GasMeter-${date:0:4}${date:5:2}${date:8:2}
  echo `date "+%Y/%m/%d %X"` $meter $jeecount > /etc/heyu/GasMeter
  mv /etc/heyu/JeeGasCounterDecoder.skipped /etc/heyu/archive/JeeGasCounterDecoder-${date:0:4}${date:5:2}${date:8:2}.skipped
  touch /etc/heyu/JeeGasCounterDecoder.skipped
  echo '----------JeeGasCounterDecoder.txt--------------' > /tmp/email
  /usr/bin/tail /etc/heyu/JeeGasCounterDecoder.txt -n1 >> /tmp/email  
  mv /etc/heyu/JeeGasCounterDecoder.txt /etc/heyu/archive/JeeGasCounterDecoder-${date:0:4}${date:5:2}${date:8:2}.txt
  touch /etc/heyu/JeeGasCounterDecoder.txt
  echo '----------JeeGasCounterDecoder.dat--------------' >> /tmp/email
  /usr/bin/tail /etc/heyu/JeeGasCounterDecoder.dat -n1 >> /tmp/email
  mv /etc/heyu/JeeGasCounterDecoder.dat /etc/heyu/archive/JeeGasCounterDecoder-${date:0:4}${date:5:2}${date:8:2}.dat
  touch /etc/heyu/JeeGasCounterDecoder.dat  
echo '------------jeebash.err------------' >> /tmp/email
/usr/bin/tail /etc/heyu/jeebash.err >> /tmp/email
rm /etc/heyu/jeebash.err
echo '------------receive.log------------' >> /tmp/email
/bin/cat /tmp/receive.log >> /tmp/email
echo '------------df -h------------' >> /tmp/email
/bin/df -h >> /tmp/email
echo '------------------------' >> /tmp/email

  /usr/sbin/ssmtp itaide@googlemail.com << EOF
To: itaide@googlemail.com
Date: `date`
Subject: Sandhurst Energy Management `date -R`
Body:
------------------------
`/bin/cat /tmp/email`
------------------------

EOF
#
  echo "Log Cleared by script on `date`" > /tmp/receive.txt
fi
echo `date "+%Y/%m/%d %X"` $# $@ >> /etc/heyu/JeeGasCounterDecoder.txt
echo `date "+%Y/%m/%d %X"` $node $sequence $count $timer $temperature $voltage >> /etc/heyu/JeeGasCounterDecoder.dat
if [ "$count" -lt "$jeecount" ]; then
  let jeecount=$count     # JeeNode integer overflow or restart
  let count=$count+1      # Resync and assume 1 unit used
  echo "DEBUG1:($date)($time),M=($meter),C=($count),J=($jeecount)" >> /etc/heyu/DEBUG1-INC
fi
if [ "$count" -gt "$jeecount" ]; then
  let "inc=($count-$jeecount)*10"                        # Multiplied by 10 in Sandhurst *10
  let "newmeter=$meter+$inc"
  echo `date "+%Y/%m/%d %X"` $newmeter $count >> /etc/heyu/GasMeter   # Append to avoid flash wear on RPi SD card
fi
#  echo "DEBUG2:($date)($time),M=($meter),C=($count),J=($jeecount),I=($inc)" >> /etc/heyu/DEBUG2-INC
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
# REMOVED from end of line below ,{"id":"voltage", "current_value":"'$voltage'"},{"id":"meter", "current_value":"'$newmeter'"}
echo '{ "version":"1.0.0","datastreams":[ {"id":"temperature", "current_value":"'$temperature'"}]}' > /tmp/cosm.json
/usr/bin/curl --request PUT --data-binary @/tmp/cosm.json --header "X-ApiKey: aykgZFSvrNzXgHpIrDVF4Zhat3OSAKx1SUJ1bDlTMU5IQT0g" http://api.cosm.com/v2/feeds/84359 >/dev/null 2>&1
#echo $count $meter $jeecount >> count.txt
#################################################################
rm -rf "${LOCKDIR}"
else
 echo `date "+%Y/%m/%d %X"` $# $@ >> /etc/heyu/JeeGasCounterDecoder.skipped
fi
#################################################################
