# NFS device trees are GENERATED. Each clock variant differs only in
# timebase-frequency, which must equal the rate the RTL's CLINT really ticks at -- so it
# is computed from PROBE_CLK_DIV8 by the same function that checks it, rather than typed
# in once per file. Five hand-copied variants had drifted to identical-except-one-number,
# all carrying the same stale model string.
#
# THE CLOCK IS ONE VALUE, NOT ONE PER RULE. timebase-frequency must equal the rate the
# CLINT really ticks at, so a tree generated for a different PROBE_CLK_DIV8 is not a
# variant -- it is a wrong DTB, and every kernel deadline, TCP timeout and NFS retry is
# off by the ratio. It was written out per-rule, and two of the three still said 72 after
# the shipping clock went to 48, so `make dtbs` would silently put the 111 MHz timebase
# back into the tree that actually gets booted. The variants differ in the UART mode and
# in nothing else. Keep this equal to the DIV8 src/lint.sh checks and the one the bit is
# built with (`make bit ... PROBE_CLK_DIV8=48`).
DIV8  ?= 48
DTC   ?= dtc
GEN   := ../../tools/check-dts-timebase.py
TMPL  := ubuntu-nfs.dts.in

%.dtb: %.dts
	$(DTC) -I dts -O dtb < $^ > $@

.PHONY: dtbs
dtbs: ubuntu-nfs.dtb ubuntu-nfs-166.dtb ubuntu-nfs-polled.dtb

ubuntu-nfs.dts: $(TMPL) $(GEN)          # the one to boot
	$(GEN) --gen $(DIV8) $(TMPL) $@
ubuntu-nfs-166.dts: $(TMPL) $(GEN)      # kept for the name; identical while DIV8=48
	$(GEN) --gen 48 $(TMPL) $@
ubuntu-nfs-polled.dts: $(TMPL) $(GEN)   # UART polled instead of interrupt-driven
	$(GEN) --gen $(DIV8) --polled $(TMPL) $@
