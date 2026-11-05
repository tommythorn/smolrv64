# THE IDENTITY STAMP: a routed design becomes another commit's by its INIT bits alone.
#
#   vivado -mode batch -source tools/stamp-identity.tcl -tclargs <routed.dcp> <commit> <dirty> <stamp> <outdir>
#
# The build's identity -- the commit (8 hex digits), the dirty bit and the stamp (14 digits,
# YYYYMMDDhhmmss) -- is the INIT value of kept flops: rv_soc_top's bid_q {dirty, commit, stamp}
# and csr_file's mimpid_q (the commit). Nothing else in the design depends on it, so two builds of
# one set of sources are the same netlist, placement and routing (docs/rtl-rules.md I2), and the
# design a probe routed is the gate's once these INITs say the gate's commit. Writes the routed
# timing summary, identity.txt (the INITs read back) and, when the design meets timing,
# <outdir>/<project>.bit (the 6th argument, rk_xcku5p by default); a design that does not prints STAMP-REFUSED and writes no bitstream.
lassign $argv dcp commit dirty stamp out proj
if {$proj eq ""} { set proj rk_xcku5p }   ;# the bitstream's name: the board's project
foreach {what v re} [list commit $commit {^[0-9a-f]{8}$} dirty $dirty {^[01]$} stamp $stamp {^[0-9]{14}$}] {
    if {![regexp $re $v]} { error "stamp-identity: $what '$v' is not of the form $re" }
}
open_checkpoint $dcp
set bid [get_cells -hier -filter {NAME =~ "*/bid_q_reg[*]"}]
set mim [get_cells -hier -filter {NAME =~ "*/mimpid_q_reg[*]"}]
if {[llength $bid] != 97 || [llength $mim] != 32} {
    error "stamp-identity: $dcp has [llength $bid] bid_q and [llength $mim] mimpid_q flops (97 and 32 expected): no identity flops to stamp"
}
# the bit each flop holds: bid_q = {dirty (bit 96), commit (95:64), stamp (63:0)}, mimpid_q =
# commit. Vivado's Tcl integers are 64 bits wide, so each field is read on its own (the hex digits
# are checked above, so building the literals by concatenation is safe).
set c [expr "0x$commit"]
set s [expr "0x$stamp"]
proc bid_bit {i} {
    global c s dirty
    if {$i < 64} { return [expr {($s >> $i) & 1}] }
    if {$i < 96} { return [expr {($c >> ($i - 64)) & 1}] }
    return $dirty
}
proc mim_bit {i} { global c; return [expr {($c >> $i) & 1}] }
# set every flop of a group from its bit function, then read them back as a 0/1 string, MSB first
proc stamp_cells {cells bitf} {
    foreach cell $cells {
        if {![regexp {\[([0-9]+)\]$} [get_property NAME $cell] -> i]} { error "stamp-identity: no bit index in [get_property NAME $cell]" }
        set_property INIT [expr {[$bitf $i] ? "1'b1" : "1'b0"}] $cell
    }
}
proc read_cells {cells} {
    set bits [lrepeat [llength $cells] 0]
    foreach cell $cells {
        regexp {\[([0-9]+)\]$} [get_property NAME $cell] -> i
        lset bits end-$i [expr {[get_property INIT $cell] eq "1'b1"}]
    }
    return [join $bits ""]
}
proc want_bits {n bitf} {
    set bits ""
    for {set i [expr {$n - 1}]} {$i >= 0} {incr i -1} { append bits [$bitf $i] }
    return $bits
}
stamp_cells $bid bid_bit
stamp_cells $mim mim_bit
set got_bid [read_cells $bid];  set want_bid [want_bits 97 bid_bit]
set got_mim [read_cells $mim];  set want_mim [want_bits 32 mim_bit]
if {$got_bid ne $want_bid || $got_mim ne $want_mim} {
    error "stamp-identity: read back bid_q $got_bid mimpid_q $got_mim, want $want_bid $want_mim"
}
# the fields as read back, in hex: dirty, commit, stamp; and mimpid_q
proc bits_hex {bits} { set v 0; foreach b [split $bits ""] { set v [expr {($v << 1) | $b}] }; return [format %x $v] }
set rb_dirty  [string index $got_bid 0]
set rb_commit [bits_hex [string range $got_bid 1 32]]
set rb_stamp  [bits_hex [string range $got_bid 33 96]]
set rb_mim    [bits_hex $got_mim]
file mkdir $out
set f [open $out/identity.txt w]
puts $f "commit $commit dirty $dirty stamp $stamp\nread back: bid_q dirty $rb_dirty commit $rb_commit stamp $rb_stamp, mimpid_q $rb_mim\nfrom $dcp"
close $f
report_timing_summary -file $out/timing_summary.rpt
# A bitstream only from a design that meets timing, as build.tcl writes one (the stamp changes no
# path; this refuses a source that never met).
set wns [get_property SLACK [lindex [get_timing_paths -setup -max_paths 1] 0]]
set whs [get_property SLACK [lindex [get_timing_paths -hold -max_paths 1] 0]]
if {$wns < 0 || $whs < 0} { puts "STAMP-REFUSED: WNS=$wns WHS=$whs"; exit 1 }
write_bitstream -force $out/$proj.bit
puts "STAMP-DONE $commit dirty=$dirty stamp=$stamp WNS=$wns"
