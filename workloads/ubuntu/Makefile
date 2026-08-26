# NFS device trees are GENERATED. Each clock variant differs only in
# timebase-frequency, which must equal the rate the RTL's CLINT really ticks at -- so it
# is computed from PROBE_CLK_DIV8 by the same function that checks it, rather than typed
# in once per file. Five hand-copied variants had drifted to identical-except-one-number,
# all carrying the same stale model string.
DTC   ?= dtc
GEN   := ../../tools/check-dts-timebase.py
TMPL  := ubuntu-nfs.dts.in

%.dtb: %.dts
	$(DTC) -I dts -O dtb < $^ > $@

.PHONY: dtbs
dtbs: ubuntu-nfs.dtb ubuntu-nfs-166.dtb ubuntu-nfs-polled.dtb

ubuntu-nfs.dts: $(TMPL) $(GEN)          # DIV8=72 -> 111.11 MHz
	$(GEN) --gen 72 $(TMPL) $@
ubuntu-nfs-166.dts: $(TMPL) $(GEN)      # DIV8=48 -> 166.67 MHz
	$(GEN) --gen 48 $(TMPL) $@
ubuntu-nfs-polled.dts: $(TMPL) $(GEN)   # DIV8=72, UART polled instead of interrupt-driven
	$(GEN) --gen 72 --polled $(TMPL) $@
