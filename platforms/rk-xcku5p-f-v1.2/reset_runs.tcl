# Reset synth_1/impl_1 after a killed build. A run terminated mid-flight leaves its
# directory half-written; the NEXT build inherits it and dies ~30 minutes in with an error
# that points at the new build rather than the old one. Always reset after a kill.
set xpr [file normalize [file join [file dirname [info script]] rk_xcku5p.xpr]]
open_project $xpr
# every run, the IPs' out-of-context runs too: a run killed mid-flight blocks the next launch of
# anything that depends on it until it is reset
foreach r [get_runs -quiet] {
    puts "resetting $r (was: [get_property STATUS $r])"
    reset_run $r
}
puts "runs reset."
