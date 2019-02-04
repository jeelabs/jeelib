#!/bin/bash
args=("$@")	# Assign to array
let "t=${args[0]}"
if [ -f /tmp/avgOutside ] ;
	then
		read ts lastAvg r1 r2 r3 r4 r5 r6 r7 r8 r9 overflow < /tmp/avgOutside
	else
		let "ts=0"
		let "lastAvg = 0"
		let "r1=r2=r3=r4=r5=r6=r7=r8=r9=$t"
fi
printf -v now '%(%s)T' -2	# Epoc when shell started
if (( $now > $ts+59 )); then	# Max one sample per minute
	let "Avg=$t+$r1+$r2+$r3+$r4+$r5+$r6+$r7+$r8+$r9+5"
	let "Avg=$Avg/10"
	if [ "$lastAvg" -ne "$Avg" ]; then
		echo `date "+%d/%m/%Y %X"` $@ $Avg >> /etc/heyu/Outside.txt
		echo "$now $Avg $Avg $Avg $Avg $Avg $Avg $Avg $Avg $Avg $Avg $Avg" > /tmp/avgOutside
		echo "$now $Avg $t $r1 $r2 $r3 $r4 $r5 $r6 $r7 $r8 $r9" >> /tmp/DEBUGavgOutside
		if [ "$Avg" -gt "17" ]; then
			/usr/bin/jee  17,130p	# 46°
		elif [ "$Avg" -eq "17" ]; then
			/usr/bin/jee 17,135p	# 47°
		elif [ "$Avg" -eq "16" ]; then
			/usr/bin/jee 17,143p	# 48.6°
		elif [ "$Avg" -eq "15" ]; then
			/usr/bin/jee 17,155p	# 51° Reasonable 14/1/19
		elif [ "$Avg" -eq "14" ]; then
			/usr/bin/jee 17,160p	# 52° Reasonable 8/1/19
		elif [ "$Avg" -eq "13" ]; then
			/usr/bin/jee 17,163p	# 52.6° Tuned 9/1/19
		elif [ "$Avg" -eq "12" ]; then
			/usr/bin/jee 17,175p	# 55.0° Reasonable 4/1/19
		elif [ "$Avg" -eq "11" ]; then
			/usr/bin/jee 17,177p	# 55.4° Reasonable 4/1/19
		elif [ "$Avg" -eq "10" ]; then
			/usr/bin/jee 17,178p	# 55.6°	Reasonable 23/1/19
		elif [ "$Avg" -eq "9" ]; then
			/usr/bin/jee 17,184p	# 56.8°
		elif [ "$Avg" -eq "8" ]; then	# -2
			/usr/bin/jee 17,189p	# 57.8°
		elif [ "$Avg" -eq "7" ]; then	# -4
			/usr/bin/jee 17,194p	# 58.8°
		elif [ "$Avg" -lt "6" ]; then
			/usr/bin/jee 17,199p	# 59.8°
		fi
	else
		echo "$now $Avg $t $r1 $r2 $r3 $r4 $r5 $r6 $r7 $r8 $r9" > /tmp/avgOutside
	fi
fi
#