# Virtio Device Plan

This plan makes virtio-mmio the shared device substrate for networking and fast
storage, while keeping the first working slice small enough to debug on the
current RK-XCKU5P-F target.

## Direction

Use Linux's existing virtio drivers instead of cloning vendor storage and
network devices.  The FPGA exposes one or more `virtio,mmio` devices through
the existing MMIO region, and device backends move data between DDR4 and the
real hardware:

- `virtio-net` backed by the Ethernet MAC/PHY.
- `virtio-blk` backed by the SD card engine, later replacing the slow
  `mmc-spi-slot` root device with `/dev/vda`.
- Display stays separate at first: use a simple scanout framebuffer and
  `simple-framebuffer`/`simpledrm`; consider `virtio-gpu` only after network
  and block are stable.

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
   enough for the first network implementation.

2. Add a DMA coherency story.

   Implement RISC-V Zicbom cache block management instructions:
   `cbo.clean`, `cbo.inval`, and `cbo.flush`.  Advertise `zicbom` plus
   `riscv,cbom-block-size = <64>` in the CPU DT node once the instructions are
   implemented.  For early bring-up, reserve an uncached/coherent DMA window for
   vrings if that is faster than relying on full Linux non-coherent DMA support.

3. Add a minimal virtio-mmio shell.

   Implement the virtio-mmio register block and split virtqueue support.  The
   first shell can expose a dummy network device and should be able
   to:

   - report `MagicValue`, `Version`, `DeviceID`, `VendorID`;
   - negotiate a small feature set;
   - accept queue PFN/ready setup from Linux;
   - raise a PLIC interrupt when software kicks the queue.

4. Bring up virtio-net over the Ethernet MAC/PHY.

   Reuse the virtqueue/DMA/interrupt groundwork.  Start with one RX queue, one
   TX queue, no checksum/TSO/GSO offloads, and fixed MTU.  Hardware owns PHY
   bring-up and MDIO/link handling; Linux sees a normal virtio-net NIC.

5. Add a fake virtio-blk backend after networking works.

   Before touching the SD command engine, return deterministic data from a small
   RAM or ROM-backed block device.  This proves Linux enumeration, queue parsing,
   descriptor walking, used-ring writes, interrupts, and cache maintenance.

6. Replace the fake block backend with the SD backend.

   Add a hardware SD block engine behind the virtio-blk frontend.  The first
   useful version only needs normal 512-byte read/write requests.  It can use
   the existing SPI pins initially, then evolve toward faster native SD later if
   the virtio interface is already working.

7. Add display output separately, then revisit virtio-gpu.

   Use a linear framebuffer in DDR with a scanout engine first.  Advertise it as
   `simple-framebuffer` or let `simpledrm` bind through firmware/DT.  Only add
   `virtio-gpu` after network and block are reliable.

## First Milestone

The first hardware-visible milestone is networking, not fast storage.  It is:

```text
Linux boots from the existing SD/MMC path
  -> sees one virtio-mmio network device in DT
  -> probes virtio-net
  -> configures RX/TX queues
  -> exchanges a minimal Ethernet frame through the MAC/PHY
  -> receives a completion interrupt
```

That milestone proves the shared virtqueue/DMA/interrupt mechanism on the
highest-value external interface before the SD backend and block-root work
become entangled.

## Current Progress

- A standalone `virtio_mmio` register shell exists in `src/virtio_mmio.v`.
- The RK top instantiates a dormant block-device shell at `0x10002000`.
- The RK top routes the shell interrupt to PLIC source 11.
- A two-master AXI arbiter exists for routing both the CPU and device DMA to
  DDR4.  The active RK build is reintroducing it first with the second master
  tied off so timing can be checked before any real device backend is connected.
- A fake RAM-less virtio-blk backend can walk one split-virtqueue request,
  return deterministic read data, discard writes, update the used ring, and
  raise the virtio interrupt.  It is currently compiled out of the RK top
  because Ethernet is the first real virtio target and the first integrated
  block version did not meet timing at 333 MHz.
- The RK top keeps the virtio block shell dormant (`DeviceID = 0`) while the
  active backend work shifts to virtio-net.  The DT node remains disabled until
  the cache/DMA coherency path is ready enough for Linux to safely probe it.
- `workloads/ubuntu/ubuntu.dts` contains a matching disabled DT node.  Enable
  it only after a backend can complete queue requests.

## Resume Note: Stop Chasing Noncoherent Virtio

As of commit `366ce99` (`Instrument virtio net TX path`), hardware testing
proved the virtio-net TX failure is a cache coherency problem, not an interrupt
delivery problem.

Observed on the programmed board:

- Ubuntu booted with virtio-net enumerated.
- `systemd-networkd` brought `eth0` up.
- Linux repeatedly reported:
  `virtio_net virtio0 eth0: NETDEV WATCHDOG: transmit queue 0 timed out`.
- The debug overlay at `0x10003f00` showed:
  - `debug_status = 0x0F000060`
  - `notify_count = 1`
  - `read_avail_count = 0x100`
  - `empty_avail_count = 0x100`
  - `read_ring_count = 0`
  - `complete_count = 0`
  - `irq_count = 0`
  - `dma_error_count = 0`

Interpretation:

Linux kicks TX queue 1, the RTL backend repeatedly reads the avail ring, but it
always sees `avail.idx == 0`. The backend never reaches descriptor-ring reads,
used-ring writes, or interrupts. That means the device DMA path is reading stale
DDR contents while Linux's updated virtqueue state is resident in the CPU cache.

Do not spend more time trying to fix this as an IRQ, queue-notify, or
virtio-mmio register bug. The current TX-drop backend is useful only as a
coherency reproducer and smoke test.

Next direction:

1. Build a coherent DMA path before extending virtio-net or adding virtio-blk.
2. Prefer a hardware coherent-IO path where DMA reads probe/read dirty CPU
   cache lines and DMA writes update or invalidate resident CPU lines.
3. A fallback is correct Zicbom/noncoherent DMA, but only if Linux's
   `cbo.clean`, `cbo.flush`, and `cbo.inval` paths are verified to make
   virtqueue updates visible before device DMA.
4. Keep the debug overlay until coherent DMA is working; it gives a cheap
   pass/fail signal:
   `read_ring_count` and `complete_count` must advance after `notify_count`.
5. Once coherent DMA is available, retest the same bit-level scenario before
   adding the real Ethernet MAC/PHY data path.

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
