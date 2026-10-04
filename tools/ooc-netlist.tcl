# Out-of-context synthesis of rv_soc_top with the shipping build's options and defines (mirror
# build.tcl when those change), then its functional netlist: the top's ports survive OOC, which
# they do not in the full design's rebuilt hierarchy. args: <repo_root> <tag> <outdir>  (rule F5)
set root [lindex $argv 0]; set tag [lindex $argv 1]; set out [lindex $argv 2]
set xpr [file join $root platforms rk-xcku5p-f-v1.2 rk_xcku5p.xpr]
set fh [open $xpr r]; set txt [read $fh]; close $fh
set vfiles {}; set svfiles {}
foreach {m p} [regexp -all -inline {File Path="\$PPRDIR/\.\./\.\./([^"]+)"} $txt] {
   set f [file normalize [file join $root $p]]
   if {![file exists $f]} continue          ;# the .xpr outlives its sources; read what is there
   if {[info exists seen($f)]} continue
   set seen($f) 1
   if {[string match *.sv $f]} { lappend svfiles $f } elseif {[string match *.v $f]} { lappend vfiles $f }
}
# The core's own files, as build.tcl's configure_smolrv64_sources adds them at build time: every
# core/*.v but the benches. The committed .xpr is only rewritten by a build, so a module new to
# core/ (smolrv64_fring.v, rv_dcache.v, rv_icache.v) is missing from it in a clean checkout, and
# this synthesis died on it ("module 'smolrv64_fring' not found") before the build had a chance.
foreach f [lsort [glob -nocomplain [file join $root core *.v]]] {
   set f [file normalize $f]
   if {[regexp {^tb_} [file tail $f]] || [info exists seen($f)]} continue
   set seen($f) 1; lappend vfiles $f
}
puts "sources: [llength $vfiles] .v, [llength $svfiles] .sv from $xpr and core/"
set incdirs {}
foreach d [list [file join $root core] [file join $root src] [file join $root src generated]] { if {[file isdirectory $d]} { lappend incdirs $d } }
set mf [open [file join $root src cvfpu_sources.f] r]
# The manifest's files too, as build.tcl's configure_cvfpu_sources reads them: the .xpr has the
# same staleness for the FPU as for core/ (vendor/cvw/fma/fmalza.sv was missing from it).
while {[gets $mf line] >= 0} { set line [string trim $line]
   if {$line eq "" || [string match "#*" $line]} continue
   if {[string match "+incdir+*" $line]} { lappend incdirs [file normalize [file join $root src [string range $line 8 end]]]; continue }
   set f [file normalize [file join $root src $line]]
   if {[info exists seen($f)]} continue
   set seen($f) 1
   if {[string match *.sv $f]} { lappend svfiles $f } elseif {[string match *.v $f]} { lappend vfiles $f } }
close $mf
read_verilog -quiet $vfiles
read_verilog -quiet -sv $svfiles
set defs [list "MEM_BASEADDR=64'h70000000" "SOC_BOOT_HEX=\"$root/src/mem.linehex\"" \
   PROBE_CLK_DIV8=48 SMOLRV64_HW=8 "SMOLRV64_BUILD_STAMP=64'h20260906000000" "SMOLRV64_GIT_COMMIT=32'h$tag" "SMOLRV64_GIT_DIRTY=1'b0"]
puts "defines: $defs"
set opts [expr {[info exists ::env(OOC_OPTS)] ? $::env(OOC_OPTS) : "-flatten_hierarchy rebuilt -retiming -control_set_opt_threshold 16"}]
puts "synth options: $opts"
synth_design -top rv_soc_top -part xcku5p-ffvb676-2-i -mode out_of_context \
   {*}$opts -include_dirs $incdirs -verilog_define $defs \
   -generic {RESET_PC=64'h70000000}
report_utilization -file $out/util-$tag.rpt
write_checkpoint -force $out/ooc-$tag.dcp
write_verilog -mode funcsim -force $out/ooc-$tag-funcsim.v
puts "OOC-DONE $out/ooc-$tag-funcsim.v"
