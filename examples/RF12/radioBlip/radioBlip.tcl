Jm doc "Decoder for the radioBlip sketch."

proc process {device data} {
  dict extract $data raw
  set blip [bitSlicer $raw 32]
  report $data ping $blip -unit counts
  report $data age [expr {$blip/(86400/64)}] -unit days
}

Jm rev {$Id: radioBlip.tcl 7623 2011-04-27 00:52:48Z jcw $}
