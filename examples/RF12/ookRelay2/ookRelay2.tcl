Jm doc "Decoder for the ookRelay2 sketch."

proc process {device data} {
  dict extract $data raw
  array set map {
    1 VISO 2 EMX 3 KSX 4 FSX
    5 ORSC 6 CRES 7 KAKU 8 XRF 9 HEZ
  }
  while {$raw ne ""} {
    lassign [bitSlicer $raw 4 4] type size
    #puts "t $type s $size r [string length $raw] : [string map $map $type]"
    set name [Ju get map($type) OTHER]
    Decode-$name [dict replace $data name $name raw [string range $raw 1 $size]]
    set raw [string range $raw $size+1 end]
  }
  return $data
}

proc Decode-VISO {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-EMX {data} {
  # see http://fhz4linux.info/tiki-index.php?page=EM+Protocol
  # example: EMX 0211726daefb214089007900
  dict extract $data raw
  lassign [bitSlicer [bitRemover $raw 8 1] \
                      8 8 8 16 16 16] type unit seq tot avg max
  dict set data name EM$type-$unit
  dict set data twrap 65536
  report $data avg [expr {$avg * 12}] -desc "use (average)" -unit W
  report $data max [expr {$max * 12}] -desc "use (maximum)" -unit W
  report $data total $tot -desc "cumulative" -unit Wh
}

proc Decode-KSX {data} {
  dict extract $data raw
  # see http://www.dc3yc.homepage.t-online.de/protocol.htm
  # example: KSX 374309e795104a4ab54c
  # example: KSX 31ca1aabacf401
  lassign [bitSlicer [bitRemover $raw 4 1] \
                      4 4 4 4 4 4 4 4 4 4 4 4 4] \
    s f t0 t1 t2 t3 t4 t5 t6 t7 t8 t9 t10
  # the scans are a way to get rid of extra leading zero's
  switch $s {
    1 {
      set temp [scan $t2$t1$t0 %d]
      set rhum [scan $t5$t4$t3 %d]
      if {$f & 0x8} { set temp -$temp }
      set unit [expr {$f & 0x7}]
      dict set data name S300-$unit
      report $data temp $temp -desc "temperature" -unit °C -scale 1
      report $data humi $rhum -desc "humidity" -unit % -scale 1
    }
    7 {
      # Log ksx {<$s$f-$t10$t9$t8-$t7$t6$t5-$t4$t3-$t2$t1$t0>\
      #           [binary encode hex [bitRemover $raw 4 1]]}
      set temp [scan $t2$t1$t0 %d]
      set rhum [scan $t4$t3 %d]
      set wind [scan $t7$t6$t5 %d]
      set rain [expr {256 * $t10 + 16 * $t9 + $t8}]
      if {$f & 0x8} { set temp -$temp }
      set rnow [expr {$f & 0x2 ? 1 : 0}]
      dict set data name KS300
      dict set data rwrap 2048
      report $data temp $temp -desc "temperature" -unit °C -scale 1
      report $data humi $rhum -desc "humidity" -unit %
      report $data wind $wind -desc "wind speed" -unit km/h -scale 1
      report $data rain $rain -desc "rain (cumulative)" -unit (0-2047)
      report $data rnow $rnow -desc "raining now" -unit (0-1)
    }
    default {
      set cleaned [bitRemover $raw 4 1]
      report $data hex [binary encode hex $cleaned]
    }
  }
}

proc Decode-FSX {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-ORSC {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-CRES {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-KAKU {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-XRF {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-HEZ {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

proc Decode-OTHER {data} {
  dict extract $data raw
  report $data hex [binary encode hex $raw]
}

Jm rev {$Id: ookRelay2.tcl 7623 2011-04-27 00:52:48Z jcw $}
