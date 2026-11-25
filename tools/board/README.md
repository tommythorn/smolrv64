# Board-side scripts

These run on a board from its `/var/tmp`, which is on its NFS root (coffee's
`/srv/ubuntu-root-2g/var/tmp` and `/srv/ubuntu-root-4g/var/tmp`): copy them to both. `gb5-run.sh <rtl>` is the measured Geekbench 5 run
(`IV=<ms>` adds per-interval counts), `gb5-probe.sh <tag>` the hang probe that runs beside
`perflog.sh`, `quiet-board.sh` masks what a benchmark machine does not need, and `errlog.py`
reads the SoC's integrity log. Both runners wait for an idle machine (`idle-wait.sh`).
`gb6-trace.sh <rtl>` is GB6's single-core pass with the counters sampled every 10 s beside the stamped console, and `gb6-split.py` (on coffee) cuts the samples into per-subtest Top-Down rows. `membench/` is the memory set (`make` there, then `./run.sh <tag>`): `chase2`, a pointer chase that
reads the core's own counters around its timed loop, with clamchowder's MemoryLatency and
MemoryBandwidth and dsheffie's mem-lat, all built on the board with the full `-march`.
`membench/summary.sh` is the short set `gb5-run.sh` runs before every Geekbench run, its figures in
`/var/tmp/gb5-<rtl>-mem.txt`: `chase2` at 16 KiB (a D$ hit, load-use), 256 KiB (beyond the D$, the
L2), 4 MiB (the dTLB's second level as well, and beyond the L2) and 64 MiB (a page walk on every
load), and `bw`'s sequential 8-byte reads and writes over 32 KiB (in the D$), 512 KiB (beyond it,
the L2) and 64 MiB, all in core cycles. `membench/` goes
to the boards' `/var/tmp/membench` with the scripts.

`fpchk/fpchk.sh <board-ip> [n]` checks the FPU instruction by instruction against `tools/fpref.py` (the FMA group, the conversions, division, square root; both formats, every rounding mode, results and fflags); `fpchk.c` is compiled on the board. 0 mismatches is the pass.
