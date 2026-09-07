# Out-of-context synthesis of rv_soc_top with the shipping build's options and defines (mirror
# build.tcl when those change), then its functional netlist: the top's ports survive OOC, which
# they do not in the full design's rebuilt hierarchy. args: <repo_root> <tag> <outdir>  (rule F5)
set root [lindex $argv 0]; set tag [lindex $argv 1]; set out [lindex $argv 2]
set xpr [file join $root platforms rk-xcku5p-f-v1.2 rk_xcku5p.xpr]
set fh [open $xpr r]; set txt [read $fh]; close $fh
set vfiles {}; set svfiles {}
foreach {m p} [regexp -all -inline {File Path="\$PPRDIR/\.\./\.\./([^"]+)"} $txt] {
   set f [file normalize [file join $root $p]]
   if {[info exists seen($f)]} continue
   set seen($f) 1
   if {[string match *.sv $f]} { lappend svfiles $f } elseif {[string match *.v $f]} { lappend vfiles $f }
}
puts "sources: [llength $vfiles] .v, [llength $svfiles] .sv from $xpr"
set incdirs {}
foreach d [list [file join $root ooo2] [file join $root src] [file join $root src generated]] { if {[file isdirectory $d]} { lappend incdirs $d } }
set mf [open [file join $root src cvfpu_sources.f] r]
while {[gets $mf line] >= 0} { set line [string trim $line]
   if {[string match "+incdir+*" $line]} { lappend incdirs [file normalize [file join $root src [string range $line 8 end]]] } }
close $mf
read_verilog -quiet $vfiles
read_verilog -quiet -sv $svfiles
set defs [list "MEM_BASEADDR=64'h70000000" "SOC_BOOT_HEX=\"$root/src/mem.linehex\"" \
   PROBE_CLK_DIV8=48 OOO2_HW=8 "SMOLRV64_BUILD_STAMP=64'h20260906000000" "SMOLRV64_GIT_COMMIT=32'h$tag" "SMOLRV64_GIT_DIRTY=1'b0"]
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
