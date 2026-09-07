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
set ooo2_dir [file join $repo_root ooo2]
set cvfpu_timing_hook [file normalize [file join [file dirname [info script]] cvfpu_timing.tcl]]
set probe_clk_check_hook [file normalize [file join [file dirname [info script]] probe_clk_check.tcl]]

puts "Opening project: $xpr"
open_project $xpr
# The .xpr is tracked and records every source ever added; a source deleted from the tree
# would otherwise stay in the fileset and stop synthesis ("No HDL sources found" at the
# 2026-09 release, when the retired cores left). Drop what no longer exists, loudly.
foreach f [get_files -quiet -of_objects [get_filesets sources_1]] {
    if {![file exists $f]} {
        puts "  dropping missing source from the fileset: $f"
        remove_files -fileset sources_1 $f
    }
}

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
    add_source_if_missing $fileset [file join $src_dir axi_two_master_arbiter.v] Verilog
    add_source_if_missing $fileset [file join $src_dir axi_single_beat_master.v] Verilog
    add_source_if_missing $fileset [file join $src_dir virtio_mmio.v] Verilog
    add_source_if_missing $fileset [file join $src_dir virtio_net.v] Verilog
    add_source_if_missing $fileset [file join $src_dir virtio_blk.v] Verilog
    add_source_if_missing $fileset [file join $src_dir sd_spi_host.v] Verilog
    add_source_if_missing $fileset [file join $src_dir crc32_d8.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_mac_tx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_mac_rx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_tx_engine.v] Verilog
    add_source_if_missing $fileset [file join $src_dir eth_rx_engine.v] Verilog
    add_source_if_missing $fileset [file join $src_dir rgmii_rx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir rgmii_tx.v] Verilog
    add_source_if_missing $fileset [file join $src_dir gmii_to_rgmii.v] Verilog

    add_source_if_missing $fileset [file join $src_dir alu.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_plic_arbiter.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_async_fifo.v] Verilog
    add_source_if_missing $fileset [file join $src_dir smolrv64_sdpram.v] Verilog
    update_compile_order -fileset $fileset
}


# The core: rv_soc_top and its modules from ooo2/. It carries its own memory subsystem
# (rv_cache / rv_l2_arbiter); the SoC devices and the DDR line bridge come from src/.
proc configure_ooo2_sources {repo_root src_dir ooo2_dir} {
    set fileset [current_fileset]
    foreach f [lsort [glob -nocomplain [file join $ooo2_dir *.v]]] {
        set b [file tail $f]
        if {[regexp {^tb_} $b]} continue
        add_source_if_missing $fileset $f Verilog
    }
    add_unique_property_value $fileset include_dirs [file normalize $ooo2_dir]
    update_compile_order -fileset $fileset
}

# Helper: put back IP output products that are not in git.
# Only the .xci is tracked for ddr4_0 -- its generated HDL and, crucially, its
# out-of-context checkpoint are not.  After a fresh clone or a `git clean -fdx` the
# checkpoint is simply gone, `read_ip` has nothing to black-box against, and synthesis
# dies two seconds into elaboration with "module 'ddr4_0' not found" -- with nothing in
# the flow offering to regenerate it.  The <ip>.dcp in the IP's output directory is the
# artifact that matters, so its absence is the test; a complete IP costs nothing here.
proc ensure_ip_products {} {
    foreach ip [get_ips -quiet] {
        set dcp [file join [get_property IP_OUTPUT_DIR $ip] $ip.dcp]
        if {[file exists $dcp]} continue
        puts "  IP $ip: output products missing ($dcp) -- regenerating."
        generate_target {instantiation_template synthesis} $ip
        synth_ip $ip
        if {![file exists $dcp]} {
            error "IP $ip: synth_ip did not produce $dcp"
        }
    }
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
# The ROM monitor boots from on-chip SRAM (lmem, 512-bit lines): bake the monitor binary
# into BRAM via $readmemh of a 64-byte-per-line hex (SOC_BOOT_HEX).
if {1} {
    set monitor_bin [file join $repo_root workloads monitor monitor.bin]
    set boot_hex    [file join $src_dir mem.linehex]
    if {![file exists $monitor_bin]} {
        error "Monitor binary missing: $monitor_bin\nRun 'make -C workloads/monitor' first."
    }
    puts "Generating boot line-hex: $boot_hex (from $monitor_bin)"
    exec python3 [file join $src_dir binline.py] $monitor_bin > $boot_hex
    configure_ooo2_sources $repo_root $src_dir $ooo2_dir
    lappend vdefines [format {SOC_BOOT_HEX="%s"} $boot_hex]
    if {[info exists env(NO_VIRTIO_NET)] && $env(NO_VIRTIO_NET) ne "" && $env(NO_VIRTIO_NET) ne "0"} {
        puts "NO_VIRTIO_NET: dropping virtio-net + eth MAC (blk-only) to relieve ui_clk congestion."
        lappend vdefines "NO_VIRTIO_NET"
    }
    if {[info exists env(PROBE_DIAG)] && $env(PROBE_DIAG) ne "" && $env(PROBE_DIAG) ne "0"} {
        puts "PROBE_DIAG: decouple probe_clk from ui_rst + LED cal/clock heartbeats (diagnostic)."
        lappend vdefines "PROBE_DIAG"
    }
    if {[info exists env(NO_SSTC)] && $env(NO_SSTC) ne "" && $env(NO_SSTC) ne "0"} {
        puts "NO_SSTC: hiding Sstc (stimecmp traps) so OpenSBI uses the CLINT timer path."
        lappend vdefines "NO_SSTC"
    }
    # RETIRED knob. It named the BUFGCE_DIV ladder (ui_clk/N); PROBE_CLK_DIV8 names the MMCM
    # output divider in eighths. The two numbers are not interchangeable and a stale
    # PROBE_CLK_DIV=3 would build 66.7 MHz while the operator believed 111. Fail loudly.
    if {[info exists env(PROBE_CLK_DIV)] && $env(PROBE_CLK_DIV) ne ""} {
        error "PROBE_CLK_DIV is retired (it meant ui_clk/N on the old BUFGCE_DIV ladder).\
 Use PROBE_CLK_DIV8 = the MMCM output divider in eighths: probe_clk = 1000 MHz * 8 / DIV8.\
 PROBE_CLK_DIV=$env(PROBE_CLK_DIV) is PROBE_CLK_DIV8=[expr {8 * $env(PROBE_CLK_DIV) * 10 / 3}]\
 (approximately -- 120/96/72 are 66.7/83.3/111.1 MHz exactly)."
    }
    # Fmax sweep knob: probe_clk = 1000 MHz * 8 / PROBE_CLK_DIV8, continuous rather than the
    # 5-rung BUFGCE_DIV ladder. The UART baud and the CLINT timebase are DERIVED from it in
    # RTL, so they cannot drift out of sync with a sweep.
    if {[info exists env(PROBE_CLK_DIV8)] && $env(PROBE_CLK_DIV8) ne ""} {
        set _d8 $env(PROBE_CLK_DIV8)
        # probe_clk is BUFGCE_DIV(ui_clk)/N, N = DIV8/24, N in 1..8.  The multiple-of-24 rule
        # is not a primitive limitation, it is a TIMING one, and it applied to the MMCM too:
        # probe_clk is derived from ui_clk, so Vivado times every probe<->ui crossing on edge
        # alignment.  Off a 3.000 ns multiple the tightest launch->capture pair collapses
        # (0.125 ns at DIV8=71) and the placer wrecks the design chasing it.  Measured
        # 2026-08-20: 64/68/70/71 gave WNS -1.080/-1.198/-2.249/-2.372 while 72 and 120 MET.
        # Refuse it here rather than let someone discover it 45 minutes into a build -- twice.
        if {![string is integer -strict $_d8] || $_d8 % 24 != 0 || $_d8 < 24 || $_d8 > 192} {
            error "PROBE_CLK_DIV8=$_d8 is not a legal probe_clk.  It must be a MULTIPLE OF 24\
 in 24..192, because probe_clk = ui_clk/(DIV8/24) and any period that is not an integer\
 multiple of ui_clk's 3.000 ns cannot be timed.  Legal: 24=333.33, 48=166.67, 72=111.11,\
 96=83.33, 120=66.67, 144=55.56, 168=47.62, 192=41.67 MHz."
        }
        set _mhz [expr {1000.0 * 8 / $_d8}]
        puts [format "PROBE_CLK_DIV8 override: probe_clk = ui_clk/%d = %.2f MHz (the shipping clock is 48 = 166.67 MHz)." [expr {$_d8 / 24}] $_mhz]
        lappend vdefines "PROBE_CLK_DIV8=$_d8"
    } else {
        # EMITTED EVEN WHEN NOT OVERRIDDEN, so the .xpr's Verilog_Define block records the
        # clock this bitstream was actually built at. That block is the only durable record
        # of a build's configuration -- it is how the 66.67 MHz builds were finally
        # identified -- and it is worthless if the shipping value is an implicit RTL default
        # that never appears in it.
        puts "probe_clk = 166.67 MHz (PROBE_CLK_DIV8=48) -- the shipping clock."
        lappend vdefines "PROBE_CLK_DIV8=48"
    }
    # Fetch window halfwords. 8 is the shipping build AND the RTL default since 2026-09-05
    # (ooo2_core.v and rv_soc_top.v agree): a 16-byte window, validated on the board as builds
    # K, M and N; sha256sum's frontend bubble went from 43% of cycles to 10%. The I$ reads it
    # as the 64-bit chunk pair (rv_cache caps the bank width), so the wide-BRAM geometry that
    # once fetched garbage on real BRAM is never built. 4 was the shipping build before.
    if {[info exists env(OOO2_HW)] && $env(OOO2_HW) ne ""} {
        # 8 is allowed since 2026-09-05: rv_cache caps its BANK width at 64 whatever RDW is
        # (a 128-bit read is the even/odd chunk pair, 16-byte aligned), so the sdpram guard is
        # never reached and the BRAM geometry is the one every bitstream has shipped with.
        if {$env(OOO2_HW) != 2 && $env(OOO2_HW) != 4 && $env(OOO2_HW) != 8} {
            error "OOO2_HW=$env(OOO2_HW): only 2, 4 or 8 elaborate; 2 is a 32-bit fetch window\
 that no bitstream should ship; odd values cannot hold a 32-bit instruction."
        }
        puts "OOO2_HW override: fetch window = $env(OOO2_HW) halfwords (the shipping build is 8)."
        lappend vdefines "OOO2_HW=$env(OOO2_HW)"
    } else {
        puts "fetch window = 8 halfwords (OOO2_HW=8) -- the shipping build."
        lappend vdefines "OOO2_HW=8"
    }
}
set build_stamp [clock format [clock seconds] -format "%Y%m%d%H%M%S"]
puts "Build stamp: $build_stamp"
lappend vdefines "SMOLRV64_BUILD_STAMP=64'h$build_stamp"
set git_commit 00000000
if {[catch {exec git -C $repo_root rev-parse --short=8 HEAD} git_result] == 0} {
    set git_commit $git_result
}
set source_dirty 0
set source_paths [list src ooo2 platforms/rk-xcku5p-f-v1.2/rk_xcku5p.srcs workloads/ubuntu workloads/tiny128]
if {[catch {exec git -C $repo_root status --porcelain --untracked-files=no -- {*}$source_paths} git_status] == 0 &&
    [string trim $git_status] ne ""} {
    set source_dirty 1
}
puts "Git commit: $git_commit"
puts "Source dirty: $source_dirty"
lappend vdefines "SMOLRV64_GIT_COMMIT=32'h$git_commit"
lappend vdefines "SMOLRV64_GIT_DIRTY=1'b$source_dirty"
# ILA_VIRTIO=1: insert an ILA on the MMIO clock-bridge ui-side (virtio-mmio access + read data),
# sampled on ui_clk. Lets us capture the driver's DeviceFeaturesSel-write / DeviceFeatures-read
# sequence on hardware and see what virtio actually returns (probe VERSION_1 -22 root-cause).
if {[info exists env(ILA_VIRTIO)] && $env(ILA_VIRTIO) ne "" && $env(ILA_VIRTIO) ne "0"} {
    puts "Enabling ILA_VIRTIO: MMIO bridge ui-side debug core (ila_virtio)."
    lappend vdefines "ILA_VIRTIO"
    if {[llength [get_ips -quiet ila_virtio]] == 0} {
        create_ip -name ila -vendor xilinx.com -library ip -module_name ila_virtio
        set_property -dict [list \
            CONFIG.C_NUM_OF_PROBES {6} \
            CONFIG.C_PROBE0_WIDTH {12} \
            CONFIG.C_PROBE1_WIDTH {1} \
            CONFIG.C_PROBE2_WIDTH {1} \
            CONFIG.C_PROBE3_WIDTH {32} \
            CONFIG.C_PROBE4_WIDTH {32} \
            CONFIG.C_PROBE5_WIDTH {1} \
            CONFIG.C_DATA_DEPTH {4096} \
            CONFIG.C_INPUT_PIPE_STAGES {2} \
            CONFIG.C_ADV_TRIGGER {true} \
        ] [get_ips ila_virtio]
        generate_target {instantiation_template synthesis} [get_ips ila_virtio]
    }
}
# ILA_IRQ=1: insert an ILA on the probe-clk interrupt path (plic src-11 lifecycle bus from rv_soc_top),
# to diagnose the root-mount hang (does the virtio completion IRQ raise/pend/seip/claim/complete or
# get lost with in_service stuck?).
if {[info exists env(ILA_IRQ)] && $env(ILA_IRQ) ne "" && $env(ILA_IRQ) ne "0"} {
    puts "Enabling ILA_IRQ: probe-clk interrupt-path debug core (ila_irq)."
    lappend vdefines "ILA_IRQ"
    if {[llength [get_ips -quiet ila_irq]] == 0} {
        create_ip -name ila -vendor xilinx.com -library ip -module_name ila_irq
        set_property -dict [list \
            CONFIG.C_NUM_OF_PROBES {1} \
            CONFIG.C_PROBE0_WIDTH {18} \
            CONFIG.C_DATA_DEPTH {4096} \
            CONFIG.C_INPUT_PIPE_STAGES {2} \
            CONFIG.C_ADV_TRIGGER {true} \
        ] [get_ips ila_irq]
        generate_target {instantiation_template synthesis} [get_ips ila_irq]
    }
}
# ILA_PARITY=1: the cache data-array integrity core. Builds the design with
# -DCACHE_PARITY (a parity bit per bank word, written on every bank write and checked
# on every bank read) and captures on probe_clk with the error pulse as the TRIGGER.
# This exists because the board's corruption is silent: by the time the kernel Oopses
# we are millions of cycles past the bad read, far beyond any pre-trigger depth. The
# parity check makes the hardware say "this array just returned something other than
# what was stored" in the cycle it happens -- and if it NEVER fires while the board
# still corrupts, the cache data path is exonerated on real BRAM and the fault is
# elsewhere (core/LSU/MMU). Either outcome is decisive.
#   probe0 = {I$,D$} error pulses (TRIGGER on != 0)
#   probe1 = sticky/bank/addr snapshot   probe2 = fetch PA   probe3 = LSU state
if {[info exists env(ILA_PARITY)] && $env(ILA_PARITY) ne "" && $env(ILA_PARITY) ne "0"} {
    puts "Enabling ILA_PARITY: cache data-array integrity core (ila_parity) + -DCACHE_PARITY."
    lappend vdefines "ILA_PARITY"
    lappend vdefines "CACHE_PARITY"
    if {[llength [get_ips -quiet ila_parity]] == 0} {
        create_ip -name ila -vendor xilinx.com -library ip -module_name ila_parity
        set_property -dict [list \
            CONFIG.C_NUM_OF_PROBES {4} \
            CONFIG.C_PROBE0_WIDTH {2} \
            CONFIG.C_PROBE1_WIDTH {64} \
            CONFIG.C_PROBE2_WIDTH {64} \
            CONFIG.C_PROBE3_WIDTH {64} \
            CONFIG.C_DATA_DEPTH {4096} \
            CONFIG.C_INPUT_PIPE_STAGES {2} \
            CONFIG.C_ADV_TRIGGER {true} \
        ] [get_ips ila_parity]
        generate_target {instantiation_template synthesis} [get_ips ila_parity]
    }
}
# ILA_DEV=1: insert an ILA on the ui-clk virtio_blk backend (FSM/SD/DMA state + AXI DMA handshakes +
# SD SPI pins) to see WHERE a block request wedges (the IRQ ILA proved the device never completes).
if {[info exists env(ILA_DEV)] && $env(ILA_DEV) ne "" && $env(ILA_DEV) ne "0"} {
    puts "Enabling ILA_DEV: ui-clk virtio_blk backend debug core (ila_dev)."
    lappend vdefines "ILA_DEV"
    if {[llength [get_ips -quiet ila_dev]] == 0} {
        create_ip -name ila -vendor xilinx.com -library ip -module_name ila_dev
        set_property -dict [list \
            CONFIG.C_NUM_OF_PROBES {3} \
            CONFIG.C_PROBE0_WIDTH {22} \
            CONFIG.C_PROBE1_WIDTH {12} \
            CONFIG.C_PROBE2_WIDTH {4} \
            CONFIG.C_DATA_DEPTH {4096} \
            CONFIG.C_INPUT_PIPE_STAGES {2} \
            CONFIG.C_ADV_TRIGGER {true} \
        ] [get_ips ila_dev]
        generate_target {instantiation_template synthesis} [get_ips ila_dev]
    }
}
# ILA_CORE=1: insert an ILA on the probe-clk core/DDR-line steady state (commit pulse + line
# handshake + stuck line addr + irq bus) to localize a post-root-mount wedge with a -trigger_now
# capture of the frozen levels (DDR-path deadlock vs interrupt livelock vs userspace spin).
if {[info exists env(ILA_CORE)] && $env(ILA_CORE) ne "" && $env(ILA_CORE) ne "0"} {
    puts "Enabling ILA_CORE: probe-clk core/DDR-line debug core (ila_core)."
    lappend vdefines "ILA_CORE"
    if {[llength [get_ips -quiet ila_core]] == 0} {
        create_ip -name ila -vendor xilinx.com -library ip -module_name ila_core
        set_property -dict [list \
            CONFIG.C_NUM_OF_PROBES {4} \
            CONFIG.C_PROBE0_WIDTH {1} \
            CONFIG.C_PROBE1_WIDTH {3} \
            CONFIG.C_PROBE2_WIDTH {24} \
            CONFIG.C_PROBE3_WIDTH {18} \
            CONFIG.C_DATA_DEPTH {4096} \
            CONFIG.C_INPUT_PIPE_STAGES {2} \
            CONFIG.C_ADV_TRIGGER {true} \
        ] [get_ips ila_core]
        generate_target {instantiation_template synthesis} [get_ips ila_core]
    }
}
lappend vdefines "SMOLRV64_USE_XPM"
set_property verilog_define $vdefines [current_fileset]
configure_cvfpu_sources $repo_root $src_dir

# Synthesis — enable retiming to help close timing on long combinatorial paths
if {$step in {synth impl bit}} {
    puts "\n=== Running Synthesis ==="
    # Cap Vivado's worker threads. The design's peak (~31 GB) overshoots this box's
    # 29 GB RAM by ~2 GB, so at the default 8 threads it thrashes swap and gets
    # pressure-killed by systemd-oomd. Fewer threads -> less per-thread working memory
    # (fits in RAM) AND deterministic P&R (no more timing lottery). Env MAXTHREADS overrides.
    set maxthr [expr {([info exists env(MAXTHREADS)] && $env(MAXTHREADS) ne "") ? $env(MAXTHREADS) : 4}]
    puts "Vivado maxThreads: $maxthr"
    set_param general.maxThreads $maxthr
    # The SRAM workload is loaded with $readmemh, so the hex file contents are
    # part of the bitstream even when the RTL text is unchanged. Vivado's
    # auto-incremental synthesis can reuse BRAM INIT values from the reference
    # checkpoint and silently preserve an older monitor image.
    ensure_ip_products
    set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
    if {[lsearch [list_property [get_runs synth_1]] INCREMENTAL_CHECKPOINT] >= 0} {
        set_property INCREMENTAL_CHECKPOINT "" [get_runs synth_1]
    }
    set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]
    set more_opts ""
    # Control-set reduction. The design uses only ~62% LUTs but ~89% of CLB slices:
    # ~1670 control sets (many single-register, incl. replicated ui_cpu_reset nets)
    # scatter flops into separate slices, starving the placer of room and turning
    # 333 MHz closure into a lottery. Raising the control-set fanout threshold absorbs
    # low-fanout enables/resets into LUT logic (we have LUT headroom) -> fewer control
    # sets -> tighter slice packing -> placement freedom. Default 16, env CSOT overrides.
    set csot [expr {([info exists env(CSOT)] && $env(CSOT) ne "") ? $env(CSOT) : 16}]
    if {$csot eq "off"} {
        puts "Synth control_set_opt_threshold: (default, override disabled)"
    } else {
        puts "Synth control_set_opt_threshold: $csot"
        append more_opts " -control_set_opt_threshold $csot"
    }
    # Optional global fanout limit: the frontend enqueue path (pre_npc ->
    # rf_decode_pc_q, rf_decode_predicted_pc_q) is route-dominated by a couple of
    # high-fanout nets (fo>150). Forcing replication shortens those routes; it is
    # functionally identical (same logic, replicated drivers). Set via env.
    if {[info exists env(FANOUT_LIMIT)] && $env(FANOUT_LIMIT) ne ""} {
        puts "Synth fanout limit: $env(FANOUT_LIMIT)"
        append more_opts " -fanout_limit $env(FANOUT_LIMIT)"
    }
    if {$more_opts ne ""} {
        set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} \
            -value [string trim $more_opts] -objects [get_runs synth_1]
    }
    run_if_needed synth_1 "" 12
    puts "Synthesis complete."

    # RAM INFERENCE GATE. Whether an array is a RAM or a mux over flops is decided here,
    # silently, from its access pattern -- a broadcast read or a reset `for` loop demotes it
    # and nothing in lint, the testbenches or the cosim can see that. It shows up as slack
    # months later on a design that is 65-83% route-bound. tools/ram-manifest.txt records
    # which arrays must stay RAM; this fails the build the moment one does not.
    set _rc [catch {exec python3 [file join $repo_root tools check-ram-inference.py] \
                        [file join [get_property DIRECTORY [get_runs synth_1]] runme.log]} _rout]
    puts $_rout
    if {$_rc} { error "RAM inference regressed -- see above and docs/rtl-rules.md I7" }
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
    # MEASURED 2026-08-22, four directives over identical RTL (HEAD 2089004b) at DIV8=48:
    #   Explore                +0.054 MET      <- default
    #   AltSpreadLogic_medium  +0.047 MET
    #   ExtraTimingOpt         +0.025 MET      (the previous default)
    #   ExtraPostPlacementOpt  -0.027 FAIL
    # An 81 ps spread that STRADDLES ZERO on unchanged source.  Two consequences:
    # Explore was free margin and was the default; and a single build's WNS cannot judge
    # a change smaller than ~80 ps -- compare against two directives, not one number.
    #
    # RE-MEASURED 2026-09-01 on the tree that closes and boots (b56f3a5e + b9dbdd0 + 42508cf
    # + 69f1ac6 + the gate repairs), and the ordering INVERTED -- so the 81 ps spread above
    # is a property of one RTL state, not a ranking to carry forward:
    #   AltSpreadLogic_medium  +0.024 MET      <- default
    #   Explore                -0.180 FAIL, and -0.113 after `make physopt`
    # 204 ps between them on identical source, which is inside the 81-400 ps spread rule I2
    # warns about and larger than the whole margin.  The directive is therefore part of the
    # shipping configuration, not a sweep knob: a build that meets timing only when someone
    # remembers to pass PLACE_DIRECTIVE is the same trap OOO2_HW and PROBE_CLK_DIV8 were.
    set place_directive AltSpreadLogic_medium
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
    # probe_clk guard: assert the CDC constraints in rk_xcku5p.xdc actually bound to a
    # clock, and record the frequency the bitstream really runs at. Twice: after opt so a
    # dropped constraint fails in minutes instead of after a full place-and-route, and after
    # route so the recorded frequency is the one the bitstream is actually generated from.
    if {![file exists $probe_clk_check_hook]} {
        error "probe_clk check hook missing: $probe_clk_check_hook"
    }
    if {![llength [get_files -quiet -of_objects [get_filesets utils_1] $probe_clk_check_hook]]} {
        add_files -fileset utils_1 -norecurse $probe_clk_check_hook
    }
    set_property STEPS.OPT_DESIGN.TCL.POST   $probe_clk_check_hook [get_runs impl_1]
    set_property STEPS.ROUTE_DESIGN.TCL.POST $probe_clk_check_hook [get_runs impl_1]
    run_if_needed impl_1 "" 12
    puts "Implementation complete."
}

# Timing check after implementation
if {$step in {impl bit}} {
    set wns [get_property STATS.WNS [get_runs impl_1]]
    set tns [get_property STATS.TNS [get_runs impl_1]]
    set failing [get_property STATS.FAILING_NETS [get_runs impl_1]]
    # STATS.* can come back EMPTY even on a clean run (observed: every intra-clock WNS
    # positive, yet this gate aborted before write_bitstream because Tcl evaluates
    # {"" < 0} as TRUE, discarding a good bitstream). Treat empty as unknown and let
    # the run proceed -- the routed timing summary is the authority.
    if {$wns eq ""} {
        puts "WARNING: STATS.WNS empty -- skipping the WNS gate; check the timing summary report."
        set wns 0
    }
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
