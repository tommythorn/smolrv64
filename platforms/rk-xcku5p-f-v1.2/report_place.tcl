# report_place.tcl -- where the design actually SITS. `make place-report`
#
# Rule I7 measured every critical path at 65-83% ROUTE on a device that is only a third full
# (make util). That is a SPREAD problem, not a congestion one, so the first question is which
# clock regions each block occupies and how far apart the endpoints of the failing paths are.
set rundir rk_xcku5p.runs/impl_1
set dcp ""
foreach c {rk_xcku5p_postroute_physopted.dcp rk_xcku5p_postroute_physopt.dcp rk_xcku5p_routed.dcp rk_xcku5p_placed.dcp} {
    if {[file exists $rundir/$c]} { set dcp $rundir/$c; break }
}
if {$dcp eq ""} { error "no impl_1 checkpoint -- run `make` first" }
puts "PLACE: reading $dcp"
open_checkpoint $dcp

proc spread {label pattern} {
    set cells [get_cells -hier -quiet -filter "NAME =~ $pattern && IS_PRIMITIVE"]
    if {[llength $cells] == 0} { puts "PLACE| $label : (no cells)"; return }
    array set seen {}
    set n 0
    foreach c $cells {
        set site [get_property -quiet LOC $c]
        if {$site eq ""} continue
        set cr [get_property -quiet CLOCK_REGION [get_sites -quiet $site]]
        if {$cr eq ""} continue
        incr seen($cr); incr n
    }
    set regions [lsort [array names seen]]
    set out ""
    foreach r $regions { append out [format "%s:%d " $r $seen($r)] }
    puts [format "PLACE| %-26s %5d cells over %2d regions   %s" $label $n [llength $regions] $out]
}

spread "core (everything)"      "probe_core/core/*"
spread "  core/u_sq"            "probe_core/core/u_sq/*"
spread "  core/u_iq_i"          "probe_core/core/u_iq_i/*"
spread "  core/u_prf"           "probe_core/core/u_prf/*"
spread "  core/fe"              "probe_core/core/fe/*"
spread "  core/u_lq"            "probe_core/core/u_lq/*"
spread "caches (probe_core)"    "probe_core/u_*cache*/*"
spread "ddr4"                   "*ddr4_0*"
spread "virtio"                 "*virtio*"
spread "probe_bridge"           "probe_bridge/*"
spread "dbg_hub"                "dbg_hub/*"
puts "PLACE-DONE"
