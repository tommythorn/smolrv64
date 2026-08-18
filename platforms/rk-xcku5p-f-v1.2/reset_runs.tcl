# Reset the Vivado runs. REQUIRED after killing a build.
#
# A killed build leaves synth_1 at PROGRESS=100% STATUS=Out-of-date, and run_if_needed in
# build.tcl cannot recover it -- the next 'make bit' dies with
#     ERROR: [Common 17-69] Run 'synth_1' needs to be reset before launching.
# and it dies AFTER re-elaborating, i.e. ~30 minutes in. That cost an hour on 2026-08-18,
# twice, which is why this is a checked-in script and not something to remember.
#
#   vivado -mode batch -nojournal -nolog -source reset_runs.tcl
open_project rk_xcku5p.xpr
foreach r {impl_1 synth_1} {
    puts "before: $r PROGRESS=[get_property PROGRESS [get_runs $r]] STATUS=[get_property STATUS [get_runs $r]]"
    reset_run $r
    puts "after:  $r PROGRESS=[get_property PROGRESS [get_runs $r]]"
}
close_project
