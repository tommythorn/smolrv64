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

Most of the original constraints below have since been resolved (see Status);
they are kept for history.

- ~~The current cache is not coherent with external DMA masters.~~ RESOLVED:
  Linux maps the virtqueues/buffers `Svpbmt` NC (non-cacheable), so device DMA
  and the CPU see the same DDR. No L1 snoop/coherent-IO was needed. Verified by
  ptdump and the bare-metal `workloads/virtio-coh` test.
- ~~device DMA needs an arbitration/interconnect layer before it can reach
  DDR.~~ RESOLVED: a two-master AXI arbiter (`src/axi_two_master_arbiter.v`)
  routes core + device DMA to the DDR4 MIG.
- ~~The RK top passes `ext_irq` as zero.~~ RESOLVED: virtio completion
  interrupts are wired to PLIC sources (net = 12, block = 11).
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

The virtio-mmio **transport and DMA work end to end** — the hard parts (queues,
descriptor walking, used ring, interrupts, and DMA coherency) are done. What is
left is the real Ethernet data path (MAC + PHY) behind the working frontend.

- `src/virtio_mmio.v` — virtio-mmio register shell (MagicValue/Version/DeviceID,
  feature negotiation, queue setup, queue-notify → backend).
- `src/virtio_net_tx_drop.v` — a **working** virtio-net device whose transport is
  complete: on TX-queue notify it DMA-reads the avail ring, walks the descriptor
  ring, reads the frame from guest DDR, writes the used ring, and raises the PLIC
  interrupt. It currently **drops** the frame (no MAC), so it is a fully
  functional NIC from Linux's view minus an external wire.
- The RK top (`rk_xcku5p.v`) instantiates the net device at `0x10003000`, IRQ 12,
  with its AXI DMA master routed to DDR4 through `axi_two_master_arbiter.v`.
- **DMA coherency is solved via Svpbmt NC**: Linux maps the vrings/buffers
  non-cacheable, so device DMA and the CPU observe the same DDR. No L1 snoop was
  needed. Verified by ptdump and bare-metal `workloads/virtio-coh`.
- TX no longer wedges: `QUEUE_NUM_MAX` is 256 (virtio-net stops TX when free
  descriptors < MAX_SKB_FRAGS+2 = 19; a depth-8 ring could never wake it). The
  backend's ring indexing is parameterized by `QUEUE_SIZE` (mask, not mod-8).
- `src/virtio_blk_fake.v` — a fake RAM-less virtio-blk backend (walks one
  request, returns deterministic read data, discards writes, updates used ring,
  IRQs). Compiled out of the RK top for now; net is the first real target and the
  first integrated block version did not meet timing at 333 MHz.
- `workloads/ubuntu/ubuntu.dts` carries the matching virtio-net DT node.

## History: the TX wedge was queue size, not coherency

An earlier resume note (commit `366ce99`, "Instrument virtio net TX path")
concluded the virtio-net TX timeout was a cache-coherency problem: Linux kicked
the TX queue, the backend's avail-ring DMA always read `avail.idx == 0`, and
`read_ring_count`/`complete_count` never advanced (NETDEV watchdog forever).

**That diagnosis was wrong.** Coherency was fine — the vring is Svpbmt NC and
DMA is visible both ways (proven with ptdump and `workloads/virtio-coh`). The
real bug was the **virtqueue depth**: virtio-net stops the TX queue when free
descriptors fall below `MAX_SKB_FRAGS + 2 = 19` and only wakes it back above
that. With `QUEUE_NUM_MAX = 8` (and no `INDIRECT_DESC`) the ring could never
reach 19, so it stopped after the first packet and never restarted — hence
`notify_count = 1` and no further progress. Fixed in `ea7eec2` by bumping the
depth to 256 and masking ring indices by `QUEUE_SIZE` instead of hardcoded
mod-8. Lesson: when a virtio queue stops after exactly one packet, suspect the
driver's free-descriptor wake threshold before suspecting DMA.

## Remaining Work: the real Ethernet data path

The frontend is done; what is left is wiring `virtio_net_tx_drop`'s dropped
frames to an actual RTL8211F-CG PHY (RGMII) and adding the receive path:

1. **PHY pins + RGMII top-level ports.** Copy the `eth_txc/rxc`, `eth_txd[3:0]`,
   `eth_rxd[3:0]`, `eth_tx_ctl`, `eth_rx_ctl` constraints from the board's
   `12_UDP_TEST` design (`udp_test.srcs/.../pin.xdc`, LVCMOS18), add `mdio/mdc`
   and a PHY reset, and expose them on `rk_xcku5p`. (In progress.)
2. **RGMII 1 GbE MAC.** DDR I/O on `eth_txc`/`eth_rxc` (125 MHz), `rxc` capture
   with IDELAY alignment, GMII↔RGMII, FCS, inter-frame gap.
3. **MDIO.** Bring up the RTL8211F: link/autoneg, and the RGMII internal TX/RX
   clock delays (the 8211F's delay-config is the usual gotcha).
4. **Real TX.** Replace the "drop" with: stream the descriptor-fetched frame to
   the MAC TX FIFO; complete the used-ring entry only on MAC accept.
5. **RX path + second queue.** Provide RX buffers from the RX virtqueue, write
   received frames (after FCS check) to guest DDR via DMA, update the RX used
   ring, and raise the interrupt. Prepend the 12-byte `virtio_net_hdr`.
6. **Enable the DT node** and confirm `eth0` carries real traffic (ping/PPP).

The `0x10003f00` debug overlay (`notify_count`, `read_ring_count`,
`complete_count`, `irq_count`, `dma_error_count`) stays useful as a cheap
TX-path pass/fail signal while bringing up the MAC.

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
