# A graphics console like simmerv's `--graphics`: framebuffer, VGA scanout, virtio keyboard

Status: plan, nothing built (2026-10-03).

## Why

simmerv gained `--graphics WxH` (simmerv `d4a8abf`): a `simple-framebuffer` that Linux's simplefb
and fbcon drive, plus a virtio-input keyboard. When the display is the terminal, the terminal's
own input is translated into key presses. Tommy (2026-10-03): the board should **work exactly
like simmerv**. That means the same kernel (`simmerv/linux/fw_payload.elf`, 7.3.0-rc5, which
already has `FB_SIMPLE`, `FRAMEBUFFER_CONSOLE`, `VIRTIO_INPUT` and the fbcon fonts), the same
device-tree nodes, the same addresses and IRQ, and the same guest-visible behaviour. Only the
pixels leave through a VGA connector instead of an SDL window.

Display hardware, for now: a TinyVGA PMOD on GPIO (HS, VS, RGB222), driven at up to about
35 MHz. That is temporary, until a board with HDMI; nothing below may assume RGB222 or a
particular pixel clock.

## What simmerv does (the reference)

| | simmerv | file |
|---|---|---|
| framebuffer | `W*H*2` bytes RGB565, rounded up to a power of two, at the top of DRAM aligned to that size; with 2 GiB, 640x480 and 800x600 both take 1 MiB at `0xFFF0_0000` | `src/lib.rs` `setup_framebuffer` |
| guest view | ordinary RAM inside the `memory` node (linear map), a `/memreserve/` entry for it, and a root `framebuffer@fff00000` node: `compatible = "simple-framebuffer"`, `reg`, `width`, `height`, `stride = W*2`, `format = "r5g6b5"` | `src/fdt.rs` `embed_framebuffer` |
| DTB and initrd | moved below the framebuffer | `src/lib.rs` `usable_ram_bytes` |
| keyboard | virtio-mmio v2, device ID 18, at `0x1000_4000` (4 KiB), PLIC source **4**, node `virtio_mmio@10004000` | `src/device/virtio_input.rs`, `fdt::embed_virtio_mmio` |
| host keys | SDL scancodes are USB HID usages, mapped through Linux's `hid_keyboard[]` table; in terminal mode, bytes and xterm escape sequences are translated back into key presses; **Ctrl-C k** switches between the keyboard and serial | `sim/src/display.rs`, `sim/src/term_keys.rs`, `sim/src/nonblocknoecho.rs` |

The board's PLIC has source 4 free (`rv_soc_top.v:288`: UART 10, blk 11, net 12), and
`0x1000_4000` is unused, so the two machines can match exactly.

Linux's simplefb probe warns `simplefb: cannot reserve video memory`: the framebuffer is inside
System RAM, so `request_mem_region` fails and simplefb maps it anyway. That is expected on both
machines.

## Phase 1: the memory map and device tree

1. **Move the DTB and initrd below the framebuffer.** `ubuntu-boot.sh` loads the DTB at
   `0xFFFF_F000` and the initrd at `0xFF62_B000`; both collide with a framebuffer at
   `0xFFF0_0000`.
   - Pick addresses below the largest framebuffer we will run. 1024x768 is 1.5 MiB, rounded
     to 2 MiB at `0xFFE0_0000`, so a DTB at `0xFFDF_F000` covers every mode up to that.
   - Update the `X80000000 0 <dtb>` start command and `tools/board-gate.sh` to match.
2. **Make the memory node the full 2 GiB.** Today it is `0x7ffff000`, 4 KiB short because of
   the DTB. The framebuffer must stay inside the node so the kernel sees it as RAM, exactly as
   in simmerv. Check `tools/check-dts-memory.py` still agrees with `DRAM_TOP`.
3. **Add to `workloads/ubuntu/ubuntu-nfs.dts.in`:**
   - `/memreserve/ 0xfff00000 0x100000;`. Check that the `@...@` generator passes it through.
   - The `framebuffer@fff00000` node, as above. Width, height and stride must agree with the
     mode the firmware programs (phase 3).
   - `virtio_mmio@10004000` with `interrupts = <4>`, `interrupt-parent` the PLIC, and
     `dma-noncoherent;` like the blk and net nodes.
4. **Gate:** the board boots Ubuntu as today with the new tree, and `/proc/iomem` and `dmesg`
   show the reservation and `fb0: simplefb registered!`. There is no scanout yet, but fbcon is
   already writing into DRAM.

## Phase 2: keep the framebuffer cacheable

Linux maps the framebuffer with `ioremap_wc`, so every fbcon store carries PBMT=NC. smolrv64
honours that: an NC store goes alone through the NC slot and completes only when DDR has it
(`rv_dcache.v:70-77`; Spec:997-1003). An fbcon scroll at 800x600 rewrites about 1 MB, and may
read it back, one DDR round trip per access.

Changing the kernel would not help on its own. A write-back mapping in simplefb needs a
flusher issuing about a million `cbo.clean` a second. Do it in hardware instead, invisibly to
software:

1. **A framebuffer window, `[fb_base, fb_base + fb_size)`.** It comes from the scanout
   engine's registers (phase 3). For physical addresses inside it, ignore PBMT's NC/IO bit and
   treat the access as ordinary cacheable DRAM. Read the window from registers, never from
   PBMT. This is safe:
   - The linear-map alias is already cacheable.
   - The guest only needs its own accesses to agree, which the cache gives it.
   - WC promises no ordering against the display beyond eventual visibility.
   - `pa_mem` already marks the range speculatable.
2. **A vsync sweep of the D$.** At the start of vertical blank, walk every set and write back
   each dirty line whose physical address is inside the window; leave the lines valid.
   - Walk the cache, not the framebuffer. At most 2048 lines (128 KiB / 64 B) can be dirty,
     against 15K lines in an 800x600 framebuffer. That is about 1K tag-port cycles per frame,
     around 0.04% of the core at 60 Hz, plus the write-backs.
   - **Check first:** the skewed way. "Inside the window" needs each line's full physical
     line address. That is trivial for the direct-indexed way, but for the skewed way only if
     the tag holds every bit the skew hash consumed. If it does not, add a per-line "in
     window" bit, set on fill from the fill's physical address.
   - The sweep must share the tag port with normal lookups by the same arbitration as a fill.
     It may be paced (one set every N cycles) as long as it finishes inside vertical blank.
     800x600@56 has 25 blank lines, about 700 µs.
   - A frame may show pixels up to one frame stale, which is invisible. simmerv itself
     refreshes at 30 Hz.
3. **VHPR phase 2 (`VIRT=1`):** the window test must stay physical. Revisit when that lands.
4. **Cosim:** nothing changes. Cacheable or not, the values are identical, and simmerv has no
   cache to model.
5. **Gate:**
   - A bare-metal test that fills the window through a PBMT=NC mapping and checks that DDR
     (read back through a second, NC alias outside the window) matches after one sweep. Model
     it on `workloads/virtio-coh/virtio_coh_nc.c`.
   - Lockstep Linux boot with fbcon active: zero mismatches.
   - The board gate.

## Phase 3: timing generator and scanout DMA

1. **The timing generator is programmable.** The firmware sets the mode; Linux treats it as
   preset (simplefb never changes modes).
   - MMIO registers: h/v active, front porch, sync, back porch, sync polarities, `fb_base`,
     `fb_size`, stride, enable.
   - Put them in a new device page, e.g. `0x1000_5000`. Keep it **out of the DT**, like
     FBDIAG; Linux never touches it.
   - **The pixel clock comes from an MMCM reprogrammed over DRP** by the same firmware.
     Candidate modes, all within TinyVGA's ~35 MHz except the last two:

     | mode | pixel clock | framebuffer |
     |---|---|---|
     | 640x480@60 | 25.175 MHz | 1 MiB |
     | 800x600@56 | 36.0 MHz | 1 MiB |
     | 800x600@60 | 40.0 MHz | 1 MiB |
     | 1024x768@60 | 65 MHz | 2 MiB |
     | 1280x1024@60 | 108 MHz | 4 MiB |

   - The firmware that programs the mode must also produce matching width, height and stride
     in the DTB. Simplest: the boot script picks the mode, writes the registers, and edits the
     DTB before handing over.
2. **Scanout DMA:**
   - A read-only AXI master into DDR, so another level in front of `ddr4_arbiter_inst`, or a
     3-input arbiter.
   - A one-line BRAM buffer (stride bytes, 1600 at 800x600), refilled by bursts during the
     previous line, with an async FIFO into the pixel-clock domain.
   - Bandwidth at 800x600@60 is about 58 MB/s, trivial against DDR4.
   - The memory arbiter must give scanout bounded latency. An underrun shows as a torn line,
     which is acceptable while debugging but should be counted (an HPM event).
3. **RGB565 → RGB222:** R = `p[15:14]`, G = `p[10:9]`, B = `p[4:3]`. The pixel is a
   little-endian 16-bit word, as the guest wrote it. Keep the conversion a separate stage so the
   HDMI board can take all 16 bits.
4. **TinyVGA pins:** add HS, VS and R1 R0 G1 G0 B1 B0 to the XDC.
   - The Tiny Tapeout VGA PMOD convention is `{HS, B0, G0, R0, VS, B1, G1, R1}`. **Verify
     against the actual adapter.**
   - Use LVCMOS33 with a slow slew rate, and register the outputs in the IOB.
5. **Cosim:** scanout only reads DDR, so there is nothing to mirror.
6. **Gate:** the board shows the fbcon boot log on a monitor at 640x480, then 800x600. Ubuntu
   still reaches a login with zero faults, and IPC on the board A/B is unchanged with the
   console idle.

## Phase 4: the virtio keyboard

### 4a. The device, exactly as simmerv's

`virtio_mmio.v` instance: `DEVICE_ID = 18`, `QUEUE_COUNT = 2` (eventq 0, statusq 1),
`DEVICE_FEATURES_1 = 3` (VERSION_1 + ACCESS_PLATFORM, like blk/net, so the guest uses the DMA
API).

1. **Decode:**
   - Widen `is_virtio_r`/`is_virtio_w` in `rv_soc_top.v:215`; the mask `~0x1fff` covers only
     `0x1000_2000`-`0x1000_3fff`.
   - Add a page select in `rk_xcku5p.v:1695`.
   - **The trap:** an undecoded address below DRAM does not fault, it goes to the D$/DDR
     (`smolrv64_lsu.v:184-186`; Spec:1032 is stale). A missing decode probes garbage
     silently. Consider making undecoded sub-DRAM addresses fault, as the Spec says they do.
2. **Config space, which `virtio_mmio.v` lacks today:**
   - Byte writes at `0x100` (`select`) and `0x101` (`subsel`). Today a write is applied only
     when all four byte enables are set (`virtio_mmio.v:105-109`), so the driver's u8 writes
     would be **dropped silently**. Lanes come from the address bits (`rv_soc_top.v:226-230`).
   - Byte reads of `0x100 + n` from a small ROM, keyed by `(select, subsel)`, laid out as
     `select, subsel, size, 5 reserved, payload`. These are simmerv's `cfg_payload`:

     | select | subsel | size | payload |
     |---|---|---|---|
     | `0x01` ID_NAME | any | 16 | `"simmerv keyboard"` (or a board name; any string works) |
     | `0x03` ID_DEVIDS | any | 8 | LE u16 bustype `0x06` (BUS_VIRTUAL), vendor `0x0627`, product 1, version 1 |
     | `0x11` EV_BITS | `0x01` EV_KEY | 16 | bitmap of every keycode `hid_to_linux` can produce: 1-127, as simmerv builds it |
     | `0x11` EV_BITS | `0x14` EV_REP | 1 | `0x01`; any non-empty answer turns on the guest's autorepeat |
     | anything else | | 0 | |

   - `CONFIG_GEN` stays 0; nothing changes after reset.
3. **eventq:**
   - Pop the next 8-byte `virtio_input_event` (`le16 type, le16 code, le32 value`) from an
     event FIFO, about 32 deep.
   - When the driver has an available buffer: write the event, then the used element (len 8)
     and used idx, and raise the interrupt.
   - **An event waits in the FIFO while no buffer is available; never drop it** (simmerv's
     `service_events`).
   - Rings and buffers are reached by the device's DMA master, non-coherently, exactly like
     blk and net.
4. **statusq:** LED updates (Caps Lock). Return each available chain with len 0 and raise the
   interrupt. There are no LEDs to light; not returning them eventually starves the driver.
5. **Events:**
   - Key transitions only. The guest autorepeats (EV_REP above).
   - Every key press or release is followed by `EV_SYN/SYN_REPORT/0`, two events per
     transition.
   - Modifiers are separate presses around the key: e.g. Shift down, key down, key up, Shift
     up.
6. **IRQ:** PLIC source 4, with the same 2-FF sync as the other virtio IRQs
   (`rk_xcku5p.v:1525-1535`), and a new soc_top port wired into `src` at bit 4.
7. **DMA:** a third device master into `device_arbiter_inst`.

### 4b. The host side: the terminal is the keyboard

The board has no keyboard; its only input is the 3 Mbps UART. This is simmerv's terminal mode
(`--graphics` drawn in iTerm2), so do what simmerv does there.

1. **A UART RX demux.** Each received byte goes either to the 16550's RX FIFO, as today, or to
   a byte→key translator feeding the event FIFO. Switch with **Ctrl-C k**, as in simmerv:
   - Ctrl-C starts a command; `k` toggles the route; Ctrl-C Ctrl-C sends one Ctrl-C to
     whichever side is selected; any other byte after Ctrl-C is passed on, and the Ctrl-C is
     dropped. That is simmerv's `NonblockNoEcho::feed`.
   - The default is the keyboard once scanout is enabled. simmerv does the same whenever the
     terminal is the only keyboard.
   - **Decision for Tommy:** this steals Ctrl-C from the serial console, as simmerv's
     terminal does, so serial Ctrl-C becomes Ctrl-C Ctrl-C. The alternatives are a board
     push-button or a less common prefix byte. Pick before building.
   - The route bit is also readable and writable over MMIO, for tests.
2. **The translator**, a small FSM plus ROMs, mirroring simmerv's `sim/src/term_keys.rs`
   exactly:
   - Printable ASCII uses a 96-entry ROM giving `(keycode, shift)`, US layout: letters, digits,
     `-_ =+ [{ ]} \| ;: '" `~ ,< .> /?`, `!@#$%^&*()`.
   - Control bytes:
     - CR or LF → Enter;
     - Tab → Tab;
     - `0x7f` and `0x08` → Backspace;
     - `0x01`-`0x1a` → Ctrl + letter;
     - `0x1c`-`0x1f` → Ctrl + `\ ] ^ _`;
     - `0x00` → Ctrl + Space.
   - Escape sequences: ESC `[` or ESC `O`, then decimal parameters separated by `;`, then a
     final byte in `0x40`-`0x7e`:
     - `A`/`B`/`C`/`D` → Up/Down/Right/Left;
     - `H`/`F` → Home/End;
     - `P`-`S` → F1-F4;
     - `Z` → Shift-Tab;
     - `N~`: 1/7 → Home, 2 → Insert, 3 → Delete, 4/8 → End, 5/6 → PgUp/PgDn,
       11-15 → F1-F5, 17-21 → F6-F10, 23/24 → F11/F12;
     - a second parameter is xterm's `1 + modifiers` (Shift 1, Alt 2, Ctrl 4).
   - ESC followed by an ordinary byte → that key with Alt.
   - **A lone ESC.** simmerv calls an ESC that ends a host `read()` the Esc key. The FSM
     cannot see reads, so use a timeout: ESC with no next byte within about 1 ms is the Esc
     key. At 3 Mbps a byte is 3.3 µs, and a terminal sends a sequence in one write.
   - The keycodes are the Linux ones `hid_to_linux` produces (simmerv
     `src/device/virtio_input.rs`).
3. **One source of truth.** Generate the ROMs from simmerv's tables, or at least test the RTL
   against them:
   - Feed the same byte strings through `term_keys::translate` and through the FSM in
     simulation, and require identical event streams.
   - simmerv's unit tests (`text_and_control_characters`, `escape_sequences`) are the
     starting vectors.
4. **Later, optional:** a PS/2 keyboard on two more GPIO pins could feed the same event FIFO
   through a scancode-set-2 → keycode ROM, with the UART path kept.

### 4c. Cosim

1. **simmerv side, a small change.**
   - The cosim reference (`simmerv_create` → `Emulator::new`) has no device at `0x1000_4000`.
     The DUT's probe of the keyboard would make simmerv take a LoadAccessFault: the armed
     load value is used only after simmerv's own `load_mmio` finds a device.
   - Add `simmerv_attach_keyboard()` to `cosim/src/lib.rs` and `simmerv_cosim.h`, calling
     `Emulator::setup_keyboard`, and call it from `probe_cosim.cpp` when the DUT has the
     device.
2. **DMA mirroring.** The keyboard's DMA writes (events, used rings) must go through the TB's
   AXI slave or call `cosim_dma_write`, so they reach `simmerv_write_memory`
   (`probe_cosim.cpp:320-345`).
3. **Interrupts need nothing new.** They are forced from the DUT and claims come through the
   armed load. `probe_cosim.cpp:260` hard-codes `simmerv_set_plic_ip(…, 10, …)`, which only
   affects simmerv's own pending state.
4. **The TB needs a way to inject keys:** a DPI call or a scripted UART RX stream, so a
   lockstep run can type a command.

### 4d. Gate

- A bare-metal test: probe the config space byte by byte and compare it with simmerv's.
- Lockstep Linux boot: `dmesg` shows `input: ... as /devices/platform/10004000.virtio_mmio/...`.
  A scripted key stream reaches the tty0 shell. Zero mismatches.
- Board: with `console=ttyS0 console=tty0`, type into the VGA console from `screen` (Ctrl-C k
  toggles), including arrows, history and Ctrl-C (sent as Ctrl-C Ctrl-C). The board gate
  passes.

## Order and dependencies

1 → 3 → 2 → 4.

- Phase 1 is DT and boot-script only.
- Phase 3 makes the result visible. It works without phase 2, only slowly: every fbcon store
  is a DDR round trip.
- Phase 2 is the performance fix, measurable once there is something to look at.
- Phase 4 is independent of 2 and 3, and can go in parallel after phase 1.

## Open

- The Ctrl-C prefix decision (4b.1).
- Whether the skewed way's tag holds the full line address (2.2).
- Who programs the mode and edits the DTB: the boot script, the ROM monitor, or an OpenSBI
  platform hook (3.1).
- The TinyVGA pin order (3.4).
