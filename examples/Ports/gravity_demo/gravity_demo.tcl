Jm doc "Decoder for the Gravity Plug, both serially connected and wireless."

package require Tk

proc process {device data} {
  dict extract $data raw message
  if {$raw eq ""} {
    # no packet data, so it must be a serially connected node
    lassign $message cmd x y z
    if {$cmd ne "GRAV"} return
  } else {
    lassign [bitSlicer $raw -16 -16 -16] x y z
  }
  report $data x $x -desc "X axis" -unit g
  report $data y $y -desc "Y axis" -unit g
  report $data z $z -desc "Z axis" -unit g
  
  # the following lines are used to update the GUI display
  # this is just for demo purposes, it doesn't really belong in the driver
  variable xval
  variable yval
  variable zval
  if {![info exists xval]} {
    # Create the GUI window.
    wm title . "Gravity demo"
    foreach v {xval yval zval rho phi the} {
      variable $v
      label .$v -textvar [namespace which -var $v] -width 7
    }
    grid [label .x -text X] [label .y -text Y] [label .z -text Z] x
    grid .xval .yval .zval [label .g -text g -width 5]
    grid .rho  .phi  .the  [label .d -text ° -width 5]
  }
  set xval [format %.3f [expr {$x / 256.0}]]
  set yval [format %.3f [expr {$y / 256.0}]]
  set zval [format %.3f [expr {$z / 256.0}]]
  # see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208368036/3#3
  set rad [expr {360/(2*3.14159)}]
  variable rho [format %.2f [expr {$rad * atan2($x,sqrt($y*$y + $z*$z))}]]
  variable phi [format %.2f [expr {$rad * atan2($y,sqrt($x*$x + $z*$z))}]]
  variable the [format %.2f [expr {$rad * atan2(sqrt($x*$x + $y*$y),$z)}]]
}

Jm rev {$Id: gravity_demo.tcl 7640 2011-04-27 23:13:49Z jcw $}
