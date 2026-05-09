# Virtio Device Plan

This plan makes virtio-mmio the shared device substrate for fast storage and
networking, while keeping the first working slice small enough to debug on the
current RK-XCKU5P-F target.

## Direction

Use Linux's existing virtio drivers instead of cloning vendor storage and
network devices.  The FPGA exposes one or more `virtio,mmio` devices through
the existing MMIO region, and device backends move data between DDR4 and the
real hardware:

- `virtio-blk` backed by the SD card engine, initially replacing the slow
  `mmc-spi-slot` root device with `/dev/vda`.
- `virtio-net` backed by the Ethernet MAC/PHY.
- Display stays separate at first: use a simple scanout framebuffer and
  `simple-framebuffer`/`simpledrm`; consider `virtio-gpu` only after block and
  network are stable.

## Constraints

- The current cache is not coherent with external DMA masters.
- The current RK top connects the core AXI master directly to the DDR4 MIG, so
  device DMA needs an arbitration/interconnect layer before it can reach DDR.
- The current RK top passes `ext_irq` as zero, so virtio completion interrupts
  need PLIC source wiring.
- The first implementation should avoid changing the existing SD/MMC boot path
  until Linux can enumerate and probe a harmless virtio-mmio device.

## 7 Step Plan

1. Add a shared DDR4 access path for device DMA.

   Insert a small AXI arbiter/interconnect between the core and the DDR4 MIG.
   Start with two masters: the existing core and one device DMA master.  Keep
   the interface conservative: one outstanding transaction per device master is
   enough for the first block implementation.

2. Add a DMA coherency story.

   Implement RISC-V Zicbom cache block management instructions:
   `cbo.clean`, `cbo.inval`, and `cbo.flush`.  Advertise `zicbom` plus
   `riscv,cbom-block-size = <64>` in the CPU DT node once the instructions are
   implemented.  For early bring-up, reserve an uncached/coherent DMA window for
   vrings if that is faster than relying on full Linux non-coherent DMA support.

3. Add a minimal virtio-mmio shell.

   Implement the virtio-mmio register block and a single split virtqueue.  The
   first shell can expose a dummy or read-only block device and should be able
   to:

   - report `MagicValue`, `Version`, `DeviceID`, `VendorID`;
   - negotiate a small feature set;
   - accept queue PFN/ready setup from Linux;
   - raise a PLIC interrupt when software kicks the queue.

4. Bring up virtio-blk with a fake RAM-backed disk.

   Before touching the SD command engine, return deterministic data from a small
   RAM or ROM-backed block device.  This proves Linux enumeration, queue parsing,
   descriptor walking, used-ring writes, interrupts, and cache maintenance.

5. Replace the fake backend with the SD backend.

   Add a hardware SD block engine behind the virtio-blk frontend.  The first
   useful version only needs normal 512-byte read/write requests.  It can use
   the existing SPI pins initially, then evolve toward faster native SD later if
   the virtio interface is already working.

6. Add virtio-net over the Ethernet MAC/PHY.

   Reuse the virtqueue/DMA/interrupt groundwork.  Start with one RX queue, one
   TX queue, no checksum/TSO/GSO offloads, and fixed MTU.  Hardware owns PHY
   bring-up and MDIO/link handling; Linux sees a normal virtio-net NIC.

7. Add display output separately, then revisit virtio-gpu.

   Use a linear framebuffer in DDR with a scanout engine first.  Advertise it as
   `simple-framebuffer` or let `simpledrm` bind through firmware/DT.  Only add
   `virtio-gpu` after block and network are reliable.

## First Milestone

The first hardware-visible milestone is not fast storage.  It is:

```text
Linux boots from the existing SD/MMC path
  -> sees one virtio-mmio block device in DT
  -> probes virtio-blk
  -> configures a queue
  -> reads a known fake sector
  -> receives a completion interrupt
```

That milestone proves the shared mechanism before the SD backend and DMA
performance work become entangled.

## Current Progress

- A standalone `virtio_mmio` register shell exists in `src/virtio_mmio.v`.
- The RK top instantiates a dormant block-device shell at `0x10002000`.
- The RK top routes the shell interrupt to PLIC source 11.
- `workloads/ubuntu/ubuntu.dts` contains a matching disabled DT node.  Enable
  it only after a backend can complete queue requests.

## Proposed Address Map

Keep the existing device addresses stable:

```text
0x10000000  UART
0x10001000  existing SPI SD controller
0x10001100  SD CS GPIO
0x10001200  SD card-detect GPIO
0x10002000  virtio-mmio block device
0x10003000  virtio-mmio network device
0x10004000  display/framebuffer control
```

Use separate PLIC sources for each virtio device.  The current UART uses IRQ 10;
start virtio block at IRQ 11 and virtio net at IRQ 12 unless those conflict with
future board devices.

## Kernel Checks

Inside Ubuntu, confirm the built kernel has the needed frontend drivers:

```sh
zcat /proc/config.gz | egrep 'VIRTIO_MMIO|VIRTIO_BLK|VIRTIO_NET|RISCV_ISA_ZICBOM|RISCV_DMA_NONCOHERENT'
```

If `/proc/config.gz` is unavailable:

```sh
grep -E 'VIRTIO_MMIO|VIRTIO_BLK|VIRTIO_NET|RISCV_ISA_ZICBOM|RISCV_DMA_NONCOHERENT' /boot/config-$(uname -r)
```
