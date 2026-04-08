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

# Synthesis
if {$step in {synth impl bit}} {
    puts "\n=== Running Synthesis ==="
    run_if_needed synth_1 "" 12
    puts "Synthesis complete."
}

# Implementation
if {$step in {impl bit}} {
    puts "\n=== Running Implementation ==="
    run_if_needed impl_1 "" 12
    puts "Implementation complete."
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
