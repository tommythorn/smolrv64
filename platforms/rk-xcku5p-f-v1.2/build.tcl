# build.tcl — Vivado batch build: open project, run synth + impl + bitstream
# Usage: vivado -mode batch -source build.tcl [-tclargs <step>]
#   step: synth | impl | bit (default: bit — runs all)

set step "bit"
set force 0
foreach arg $argv {
    if {$arg eq "force"} { set force 1 } else { set step $arg }
}

set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
set repo_root [file normalize [file join [file dirname [info script]] ../..]]
set src_dir [file join $repo_root src]
set probe_dir [file join $repo_root probe]
set sram_even [file join $repo_root src mem.even]
set sram_odd  [file join $repo_root src mem.odd]
set cvfpu_timing_hook [file normalize [file join [file dirname [info script]] cvfpu_timing.tcl]]

# The sharded-OoO probe core (soc_top + DDR line/AXI bridge, gated by the
# `PROBE_CORE ifdef in rk_xcku5p.v) is now the DEFAULT.  Set PROBE_CORE=0 to
# build the legacy scalar smolrv64 instead.
set probe_core 1
if {[info exists env(PROBE_CORE)] && $env(PROBE_CORE) eq "0"} {
    set probe_core 0
    puts "PROBE_CORE=0: building the legacy scalar smolrv64 core."
} else {
    puts "Building the sharded-OoO probe core (soc_top) [default]."
}
puts "Opening project: $xpr"
open_project $xpr

proc add_source_if_missing {fileset file file_type} {
    set normalized [file normalize $file]
    if {![file exists $normalized]} {
        error "RTL source missing: $normalized"
    }
    if {[llength [get_files -quiet $normalized]] == 0} {
        add_files -norecurse -fileset $fileset $normalized
    }
    if {$file_type ne ""} {
        set_property file_type $file_type [get_files -quiet $normalized]
    }
}

proc add_unique_property_value {object property value} {
    set values [get_property $property $object]
    if {[lsearch -exact $values $value] < 0} {
        lappend values $value
        set_property $property $values $object
    }
}

proc configure_cvfpu_sources {repo_root src_dir} {
    set fileset [current_fileset]
    set cvfpu_manifest [file join $src_dir cvfpu_sources.f]
    set cvfpu_submodule [file join $repo_root third_party cvfpu src common_cells include]
    if {![file isdirectory $cvfpu_submodule]} {
        error "CVFPU submodule is missing or incomplete: $cvfpu_submodule\nRun: git submodule update --init --recursive"
    }
    if {![file exists $cvfpu_manifest]} {
        error "CVFPU source manifest missing: $cvfpu_manifest"
    }

    puts "Enabling CVFPU sources from: $cvfpu_manifest"
    set cvfpu_files {}
    set fh [open $cvfpu_manifest r]
    while {[gets $fh line] >= 0} {
        set line [string trim $line]
        if {$line eq "" || [string match "#*" $line]} {
            continue
        }
        if {[string match "+incdir+*" $line]} {
            set incdir [file normalize [file join $src_dir [string range $line 8 end]]]
            if {![file isdirectory $incdir]} {
                error "CVFPU include directory missing: $incdir"
            }
            add_unique_property_value $fileset include_dirs $incdir
            continue
        }
        if {[string match "+*" $line]} {
            error "Unsupported CVFPU manifest option: $line"
        }
        lappend cvfpu_files [file normalize [file join $src_dir $line]]
    }
    close $fh

    foreach file $cvfpu_files {
        set ext [string tolower [file extension $file]]
        set file_type ""
        if {$ext eq ".sv"} {
            set file_type SystemVerilog
        }
        add_source_if_missing $fileset $file $file_type
    }
    add_source_if_missing $fileset [file join $src_dir smolrv64_cvfpu.sv] SystemVerilog
    add_source_if_missing $fileset [file join $src_dir axi_two_master_arbiter.v] Verilog
    add_source_if_missing $fileset [file join $src_dir axi_single_beat_master.v] Verilog
    add_source_if_missing $fileset [file join $src_dir virtio_mmio.v] Verilog
    add_source_if_missing $fileset [file join $src_dir virtio_net.v] Verilog
    add_source_if_missing $fileset [file join $src_dir virtio_blk.v] Verilog
    add_source_if_missing $fileset [file join $src_dir sd_spi_host.v] Verilog
    # rgmii_mac_stub.v is no longer instantiated (replaced by gmii_to_rgmii +
    # eth_tx_engine + eth_mac_rx) but stays in the fileset/.xpr so opening the
    # project doesn't fault on a missing file; synthesis drops it (DCE).
    add_source_if_missing $fileset [file join $src_dir rgmii_mac_stub.v] Verilog
    add_source_if_missing $fileset [file join $src_dir crc32_d8.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_mac_tx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_mac_rx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_tx_engine.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_rx_engine.v] Verilog
    add_source_if_missing $fileset [file join $src_dir rgmii_rx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir rgmii_tx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir gmii_to_rgmii.v] Verilog

    add_source_if_missing $fileset [file join $src_dir alu.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_alu.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_plic_arbiter.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_async_fifo.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_l2_boundary.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_mem_engine.v] SystemVerilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_frontend.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_sdpram.v] Verilog
    add_source_if_missing $fileset [file join $src_dir regfile.v] Verilog
    add_source_if_missing $fileset [file join $src_dir fregfile.v] Verilog
    # smolrv64.v uses SystemVerilog (the always-on CV-FPU interface).
    add_source_if_missing $fileset [file join $src_dir smolrv64.v] SystemVerilog
    update_compile_order -fileset $fileset
}

# Add the sharded-OoO probe core RTL (soc_top + dependency set) to the fileset.
# Mirrors the OOC source list: all probe/*.v except testbenches, cosim-only files,
# flopwrap.v and rf_alu.v. fp_unit_synth.sv is the synth-clean FP tie-off (real
# CVFPU is deferred — task 23). ddr_line_axi/cdc + smolrv64_sdpram come from src/.
proc configure_probe_sources {repo_root src_dir probe_dir} {
    set fileset [current_fileset]
    foreach f [lsort [glob -nocomplain [file join $probe_dir *.v]]] {
        set b [file tail $f]
        if {[regexp {^tb_} $b]} continue
        if {[string match *probe* $b]} continue
        if {$b eq "flopwrap.v"} continue
        if {$b eq "rf_alu.v"} continue
        add_source_if_missing $fileset $f Verilog
    }
    add_source_if_missing $fileset [file join $probe_dir fp_unit_synth.sv] SystemVerilog
    add_source_if_missing $fileset [file join $src_dir ddr_line_axi.v] Verilog
    add_source_if_missing $fileset [file join $src_dir ddr_line_cdc.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_sdpram.v] Verilog
    # alu_ops.vh / smolrv64_fp_ops.vh live in src/; the probe set `include`s them
    # with bare names, so both probe/ and src/ must be on the include path.
    add_unique_property_value $fileset include_dirs [file normalize $probe_dir]
    add_unique_property_value $fileset include_dirs [file normalize $src_dir]
    update_compile_order -fileset $fileset
}

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
    puts "  $run_id resetting before launch."
    reset_run $run_id
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
if {$probe_core} {
    # The probe boots its monitor from on-chip SRAM (lmem, 512-bit lines). Bake the
    # monitor binary into BRAM via $readmemh of a 64-byte-per-line hex (SOC_BOOT_HEX).
    set monitor_bin [file join $repo_root workloads monitor monitor.bin]
    set boot_hex    [file join $src_dir mem.linehex]
    if {![file exists $monitor_bin]} {
        error "Monitor binary missing: $monitor_bin\nRun 'make -C workloads/monitor' first."
    }
    puts "Generating boot line-hex: $boot_hex (from $monitor_bin)"
    exec python3 [file join $src_dir binline.py] $monitor_bin > $boot_hex
    lappend vdefines "PROBE_CORE"
    lappend vdefines [format {SOC_BOOT_HEX="%s"} $boot_hex]
} else {
    foreach image [list $sram_even $sram_odd] {
        if {![file exists $image]} {
            error "SRAM init file missing: $image\nRun 'make load' or build the workload images first."
        }
    }
    puts "SRAM init even: $sram_even"
    puts "SRAM init odd:  $sram_odd"
    lappend vdefines [format {SRAM_EVENHEX="%s"} $sram_even]
    lappend vdefines [format {SRAM_ODDHEX="%s"} $sram_odd]
}
set build_stamp [clock format [clock seconds] -format "%Y%m%d%H%M%S"]
puts "Build stamp: $build_stamp"
lappend vdefines "SMOLRV64_BUILD_STAMP=64'h$build_stamp"
set git_commit 00000000
if {[catch {exec git -C $repo_root rev-parse --short=8 HEAD} git_result] == 0} {
    set git_commit $git_result
}
set source_dirty 0
set source_paths [list src platforms/rk-xcku5p-f-v1.2/rk_xcku5p.srcs workloads/ubuntu workloads/linux workloads/tiny128]
if {[catch {exec git -C $repo_root status --porcelain --untracked-files=no -- {*}$source_paths} git_status] == 0 &&
    [string trim $git_status] ne ""} {
    set source_dirty 1
}
puts "Git commit: $git_commit"
puts "Source dirty: $source_dirty"
lappend vdefines "SMOLRV64_GIT_COMMIT=32'h$git_commit"
lappend vdefines "SMOLRV64_GIT_DIRTY=1'b$source_dirty"
if {[info exists env(PC_TRACE)] && $env(PC_TRACE) ne "" && $env(PC_TRACE) ne "0"} {
    puts "Enabling PC_TRACE debug tracer."
    lappend vdefines "PC_TRACE"
}
lappend vdefines "SMOLRV64_USE_XPM"
set_property verilog_define $vdefines [current_fileset]
configure_cvfpu_sources $repo_root $src_dir
if {$probe_core} {
    configure_probe_sources $repo_root $src_dir $probe_dir
}

# Synthesis — enable retiming to help close timing on long combinatorial paths
if {$step in {synth impl bit}} {
    puts "\n=== Running Synthesis ==="
    # The SRAM workload is loaded with $readmemh, so the hex file contents are
    # part of the bitstream even when the RTL text is unchanged. Vivado's
    # auto-incremental synthesis can reuse BRAM INIT values from the reference
    # checkpoint and silently preserve an older monitor image.
    set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
    if {[lsearch [list_property [get_runs synth_1]] INCREMENTAL_CHECKPOINT] >= 0} {
        set_property INCREMENTAL_CHECKPOINT "" [get_runs synth_1]
    }
    set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
    # Optional global fanout limit: the frontend enqueue path (pre_npc ->
    # rf_decode_pc_q, rf_decode_predicted_pc_q) is route-dominated by a couple of
    # high-fanout nets (fo>150). Forcing replication shortens those routes; it is
    # functionally identical (same logic, replicated drivers). Set via env.
    if {[info exists env(FANOUT_LIMIT)] && $env(FANOUT_LIMIT) ne ""} {
        puts "Synth fanout limit: $env(FANOUT_LIMIT)"
        set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} \
            -value "-fanout_limit $env(FANOUT_LIMIT)" -objects [get_runs synth_1]
    }
    run_if_needed synth_1 "" 12
    puts "Synthesis complete."
}

# Implementation — use Performance_ExplorePostRoutePhysOpt for timing closure
if {$step in {impl bit}} {
    puts "\n=== Running Implementation ==="
    set_property STRATEGY Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
    set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
    set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
    # Place/route directives. The default strategy leaves the frontend
    # npc->rf_decode->cache_issue path at negative slack on this zero-margin
    # design (WNS -0.122); ExtraTimingOpt placement + AggressiveExplore routing
    # closes it (+0.129). These are the defaults so plain 'make bit' meets timing;
    # override via PLACE_DIRECTIVE / ROUTE_DIRECTIVE env vars for closure sweeps.
    set place_directive ExtraTimingOpt
    set route_directive AggressiveExplore
    if {[info exists env(PLACE_DIRECTIVE)] && $env(PLACE_DIRECTIVE) ne ""} {
        set place_directive $env(PLACE_DIRECTIVE)
    }
    if {[info exists env(ROUTE_DIRECTIVE)] && $env(ROUTE_DIRECTIVE) ne ""} {
        set route_directive $env(ROUTE_DIRECTIVE)
    }
    puts "Place directive: $place_directive   Route directive: $route_directive"
    set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE $place_directive [get_runs impl_1]
    set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE $route_directive [get_runs impl_1]
    if {![file exists $cvfpu_timing_hook]} {
        error "CVFPU timing hook missing: $cvfpu_timing_hook"
    }
    # Apply the timing hook before opt, place AND route. A constraint set only at
    # OPT_DESIGN.TCL.PRE does not survive into route_design's timing view or the
    # final checkpoint (each step re-reads the XDC), so the core_clk hold
    # uncertainty had no effect on route's hold-fixing. Re-applying it before
    # place and route makes route honor it (real hold margin) and persists it
    # into the saved checkpoint. The hook is idempotent.
    set_property STEPS.OPT_DESIGN.TCL.PRE   $cvfpu_timing_hook [get_runs impl_1]
    set_property STEPS.PLACE_DESIGN.TCL.PRE $cvfpu_timing_hook [get_runs impl_1]
    set_property STEPS.ROUTE_DESIGN.TCL.PRE $cvfpu_timing_hook [get_runs impl_1]
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
