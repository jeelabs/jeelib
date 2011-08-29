Jm doc "Decoder for the roomNode sketch, both serially connected and wireless."

proc process {device data} {
  dict extract $data raw message
  if {$raw eq ""} {
    # no packet data, so it must be a serially connected node
    lassign $message cmd l m h t b
    if {$cmd ne "ROOM"} return
  } else {
    # struct {
    #     byte light;     // light sensor: 0..255
    #     byte moved :1;  // motion detector: 0..1
    #     byte humi  :7;  // humidity: 0..100
    #     int temp   :10; // temperature: -500..+500 (tenths)
    #     byte lobat :1;  // supply voltage dropped under 3.1V: 0..1
    # } payload;
    lassign [bitSlicer $raw 8 1 7 -10 1] l m h t b
  }
  report $data light [expr {round($l/2.55)}] -desc "light" -unit (0-100)
  report $data moved $m -desc "motion" -unit (0-1)
  report $data humi $h -desc "humidity" -unit %
  report $data temp $t -desc "temperature" -unit °C -scale 1
  report $data lobat $b -desc "low battery" -unit (0-1)
}

Jm rev {$Id: roomNode.tcl 7638 2011-04-27 23:05:52Z jcw $}
