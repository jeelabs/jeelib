Jm doc "Decoder for the highside sketch."

proc process {device data} {
  dict extract $data raw
  while {[string length $raw] >= 6} {
    incr n
    lassign [bitSlicer $raw 16 16 16] l h a
    report $data low$n $l -unit mV
    report $data high$n $h -unit mV
    report $data avg$n $a -unit mV
    set raw [string range $raw 6 end]
  }
}

Jm rev {$Id: highside.tcl 7708 2011-06-05 20:28:21Z jcw $}
