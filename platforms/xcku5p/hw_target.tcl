# hw_target.tcl -- the one way the board scripts open a JTAG target (sourced by program.tcl,
# flash.tcl, ila_capture.tcl, ila_mem.tcl). Two boards hang off this host, their FTDI cables
# carry distinct serials (program_ftdi; hw_server shows cables that share one as a single
# target), and JTAG=<serial> in the environment picks one. Unset, the only target is opened,
# and more than one is an error that lists them: nothing ever programs or captures a board
# nobody named.
proc open_board_target {} {
    set tgts [get_hw_targets]
    if {[info exists ::env(JTAG)] && $::env(JTAG) ne ""} {
        set tgt [get_hw_targets -quiet "*/$::env(JTAG)"]
        if {[llength $tgt] != 1} { error "JTAG=$::env(JTAG) matches [llength $tgt] targets among: $tgts" }
        open_hw_target $tgt
    } elseif {[llength $tgts] == 1} {
        open_hw_target
    } else {
        error "[llength $tgts] JTAG targets ($tgts): set JTAG=<cable serial> to pick one"
    }
}
