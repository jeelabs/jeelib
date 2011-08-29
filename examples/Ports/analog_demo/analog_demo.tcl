Jm doc "Show readings from the analog plug in a little GUI window."

package require Tk

proc process {device data} {
  dict extract $data raw
  lassign [bitSlicer $raw -18] v
  report $data analog1 $v -desc "measured voltage" -unit V -scale 6

  # the following line is used to update the GUI display
  # this is just for demo purposes, it doesn't really belong in the driver
  variable value
  if {![info exists value]} {
    # create the GUI window
    wm geometry . +220+310
    wm resizable . 0 0
    # large green text on black background with a beveled edge
    label .v -textvar [namespace which -var value] -font {Times 48} -width 9 \
                -anchor e -padx 10 -bg black -fg lightgreen -bd 8 -relief sunken
    pack .v
  }  
  set value [format {%.5f V} [expr {$v * 1e-6}]]
}

Jm rev {$Id: analog_demo.tcl 7637 2011-04-27 23:05:26Z jcw $}
