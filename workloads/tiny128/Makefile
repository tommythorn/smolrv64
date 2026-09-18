SIMMERV = $(HOME)/simmerv
FW=fw_payload.bin

ifeq ($(shell uname -o),Darwin)
  RV=riscv64-elf-
  CC=clang
else # Assume Linux
  RV=riscv64-linux-gnu-
endif

.PHONY: sim cosim

# The tiny128 Linux boot in lockstep with simmerv (300 M cycles is the required length).
cosim: tiny128.dtb $(FW) tiny128.cpio
	CYC=300000000 ../../ooo2/run-ooo2-cosim-linux.sh


%: %.xz
	xz -d < $^ > $@

tiny128.cpio: tiny128.cpio.xz

tiny128.dtb: tiny128.dts
	dtc -I dts -O dtb < $^ > $@

# B6 (2026-09-17): the disk-backed cosim -- virtio-blk with a 4 MiB ext4 image that the initrd's
# S99blkcheck (blkcheck.sh, merged into tiny128.cpio as one archive by mk-blk-cpio.py) mounts,
# verifies, writes back and re-reads. Non-coherent DMA into the DUT's DDR, mirrored into simmerv.
# The kernel is at 0.24 s of guest time after 300 M cycles; userspace (and the check) needs ~1.5 G.
cosim-blk: tiny128-cosim-blk.dtb $(FW) tiny128-blk.cpio tiny128-blk.img
	DTB=$(CURDIR)/tiny128-cosim-blk.dtb INITRD=$(CURDIR)/tiny128-blk.cpio DISK=$(CURDIR)/tiny128-blk.img \
	  CYC=$${CYC:-1500000000} ../../ooo2/run-ooo2-cosim-linux.sh
tiny128-blk.cpio: tiny128.cpio blkcheck.sh mk-blk-cpio.py
	./mk-blk-cpio.py tiny128.cpio blkcheck.sh $@
tiny128-blk.img: gen-blk-img.py
	./gen-blk-img.py $@
tiny128-cosim-blk.dtb: tiny128-cosim-blk.dts
	dtc -I dts -O dtb < $^ > $@
ref: tiny128.dtb $(FW) tiny128.cpio
	(cd $(SIMMERV); cargo b -rq)
	$(SIMMERV)/target/release/simmerv_cli -m 512 -d tiny128-simmerv.dtb,0x9ff00000 $(FW),0x80000000 tiny128.cpio,0x9f52c000
