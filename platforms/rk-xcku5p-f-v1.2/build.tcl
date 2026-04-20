# build.tcl — Vivado batch build: open project, run synth + impl + bitstream
# Usage: vivado -mode batch -source build.tcl [-tclargs <step>]
#   step: synth | impl | bit (default: bit — runs all)

set step "bit"
set force 0
foreach arg $argv {
    if {$arg eq "force"} { set force 1 } else { set step $arg }
}

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
puts "Opening project: $xpr"
open_project $xpr

# Helper: launch a run only if it needs work
proc run_if_needed {run_id to_step jobs} {
    global force
    set run [get_runs $run_id]
    set needs_refresh [get_property NEEDS_REFRESH $run]
    set progress      [get_property PROGRESS      $run]
    if {!$force && !$needs_refresh && $progress eq "100%"} {
        puts "  $run_id already up to date, skipping."
        return
    }
    if {$force || $needs_refresh} {
        puts "  $run_id resetting before launch."
        reset_run $run_id
    }
    if {$to_step ne ""} {
        launch_runs $run -to_step $to_step -jobs $jobs
    } else {
        launch_runs $run -jobs $jobs
    }
    wait_on_run $run
    if {[get_property PROGRESS $run] != "100%"} {
        error "$run_id failed (progress: [get_property PROGRESS $run])"
    }
}

# Set SRAM base to 0x70000000 for this platform (below the DDR4 range at 0x80000000)
set vdefines [list "MEM_BASEADDR=64'h70000000"]
if {[info exists env(PC_TRACE)] && $env(PC_TRACE) ne "" && $env(PC_TRACE) ne "0"} {
    puts "Enabling PC_TRACE debug tracer."
    lappend vdefines "PC_TRACE"
}
set_property verilog_define $vdefines [current_fileset]

# Synthesis — enable retiming to help close timing on long combinatorial paths
if {$step in {synth impl bit}} {
    puts "\n=== Running Synthesis ==="
    set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
    run_if_needed synth_1 "" 12
    puts "Synthesis complete."
}

# Implementation — use Performance_ExplorePostRoutePhysOpt for timing closure
if {$step in {impl bit}} {
    puts "\n=== Running Implementation ==="
    set_property STRATEGY Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
    run_if_needed impl_1 "" 12
    puts "Implementation complete."
}

# Timing check after implementation
if {$step in {impl bit}} {
    set wns [get_property STATS.WNS [get_runs impl_1]]
    set tns [get_property STATS.TNS [get_runs impl_1]]
    set failing [get_property STATS.FAILING_NETS [get_runs impl_1]]
    if {$wns < 0} {
        puts "\n*** TIMING VIOLATION: WNS=${wns}ns  TNS=${tns}ns  failing_endpoints=$failing ***"
        puts "    Design will not function reliably at this clock frequency."
        error "Timing not met — fix violations before generating bitstream."
    } else {
        puts "Timing met: WNS=${wns}ns"
    }
}

# Bitstream
if {$step in {bit}} {
    puts "\n=== Generating Bitstream ==="
    set bit_dir [get_property DIRECTORY [get_runs impl_1]]
    set bit_file "$bit_dir/rk_xcku5p.bit"
    set run [get_runs impl_1]
    if {[file exists $bit_file] && ![get_property NEEDS_REFRESH $run]} {
        puts "  Bitstream already up to date, skipping."
    } else {
        launch_runs impl_1 -to_step write_bitstream -jobs 12
        wait_on_run impl_1
        if {[get_property PROGRESS $run] != "100%"} {
            error "Bitstream generation failed (progress: [get_property PROGRESS $run])"
        }
    }
    puts "Bitstream complete."
    puts "Bitstream: $bit_file"
}

close_project
puts "\nDone."
