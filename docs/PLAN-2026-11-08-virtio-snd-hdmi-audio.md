# Virtio sound and HDMI audio (2026-11-08)

Tommy: "add a virtio audio device. Assuming this fits HDMI Audio and Virtio Audio, support sample
rates 44100, 22050, and 11025, mono or stereo, 8 and 16 bits (thus a total of 12 different
configurations)."

## 1. What the guest sees

A virtio-mmio v2 sound device (ID 25, virtio 1.2 section 5.14) at `0x1000_7000`, one PLIC source,
`dma-noncoherent` like virtio-blk and virtio-input. Ubuntu's kernels ship `virtio_snd.ko`
(`/srv/ubuntu-root-hdmi/lib/modules/6.14.0-3*/kernel/sound/virtio/`), so a device-tree node is all
Linux needs.

- Config space: `jacks = 0`, `streams = 1`, `chmaps = 0`.
- One stream, output: formats `U8` and `S16` (the 8-bit format is unsigned, as WAV's is), rates
  11025, 22050 and 44100, one or two channels: the twelve configurations.
- Four virtqueues, as the driver sets them up: controlq (0), eventq (1, never used: the device
  sends no events), txq (2), rxq (3, never used: no capture stream).
- Control requests: `PCM_INFO`, `PCM_SET_PARAMS`, `PCM_PREPARE`, `PCM_RELEASE`, `PCM_START`,
  `PCM_STOP` answer `OK` (or `BAD_MSG` for a stream other than 0, a parameter outside the twelve);
  `JACK_INFO`, `JACK_REMAP` and `CHMAP_INFO` answer `NOT_SUPP` (there are none to ask about).
- Playback: each txq buffer is `{stream_id}`, the PCM bytes (one descriptor or several), then the
  device-writable `{status, latency_bytes}`. The device takes the bytes into its sample FIFO as the
  playback clock drains it, and returns the buffer once its last byte is in. A started stream is
  consumed; a stopped one waits; `PCM_RELEASE` returns every pending buffer unplayed.

## 2. One format on the wire

HDMI's L-PCM sample rates (CEA-861) are 32, 44.1, 48, 88.2, 96, 176.4 and 192 kHz: 22.05 and
11.025 kHz are not among them. So the HDMI link always carries 44.1 kHz, 16-bit, stereo, and the
device converts every configuration into it as the bytes arrive:

- 8-bit `U8` to 16-bit signed: `(x - 128) << 8`;
- mono to stereo: the sample on both channels;
- 22.05 kHz and 11.025 kHz to 44.1 kHz: each frame repeated two or four times (the 44.1 kHz family
  divides exactly; no filter, the source's own aliasing is what a 22 kHz stream has anyway).

The playback clock is 44.1 kHz from ui_clk (333.33 MHz = 1e9/3 Hz): an accumulator adds 3 * 44100
each cycle and ticks past 1e9, exact. One stereo frame leaves the FIFO each tick. On the RK boards,
which have no audio output, the frames are dropped at the same rate, so the guest's view is the
same on every board.

## 3. HDMI audio (hdmi_tx)

`hdmi_tx` today is DVI: video and control symbols, no data islands. Audio needs HDMI mode:

- **Data islands** in the horizontal blanking: a control period, the 8-pixel data island
  preamble, a 2-pixel leading guard band, packets of 32 pixels each (TERC4 on all three channels:
  channel 0 carries HSYNC, VSYNC, the header bit and the packet's first-pixel flag; channels 1 and
  2 the four subpackets' even and odd bits), a 2-pixel trailing guard band.
- **Video preamble and guard band** before every active line (8 + 2 pixels), which HDMI mode
  requires; the transmitter delays the video stream so it sees `de` rise in time.
- **Packets**: header (24 bits + BCH ECC) and four subpackets (56 bits + BCH ECC each), the ECC
  the HDMI BCH code (x^8 + x^7 + x^6 + 1, computed serially).
  - Audio Sample (0x02): up to four stereo samples a packet, 24-bit fields carrying the 16-bit
    samples, IEC 60958 V/U/C/P bits, the 192-frame block start (B) and channel status (consumer,
    PCM, 44.1 kHz, 16-bit).
  - Audio Clock Regeneration (0x01): N = 6272 (44.1 kHz's recommended value), CTS measured: the
    pixel clocks counted over each N/(128 * 44100) s of the playback clock (the clock comes from
    ui_clk, so the sink regenerates exactly the rate the device plays at, whatever the mode).
  - AVI InfoFrame (0x82) and Audio InfoFrame (0x84, 2 channels, "refer to stream header"), once a
    frame.
- The 44.1 kHz frames cross from ui_clk into the pixel clock through a small asynchronous FIFO.

At 1080p (2200 x 1125 at 57.7 Hz) a line is 15.4 us and a frame of audio 22.7 us, so one Audio
Sample packet a line (up to four frames) keeps up with room to spare; every mode the monitor sets
has at least 160 pixels of horizontal blanking, enough for a two-packet island.

## 4. Steps

1. `virtio_mmio`: four queues (virtio-snd's count), as arrays; virtio_input unchanged in behaviour.
2. `virtio_snd`: the control and tx engines, the converter, the FIFO and the playback clock;
   `tb_virtio_snd` drives the twelve configurations through a driver model and checks every
   44.1 kHz frame that leaves.
3. `hdmi_tx`: data islands, packets, video guard bands; `tb_hdmi_audio` decodes the TMDS stream
   with an independent model (TMDS and TERC4 decode, BCH check, packet parse) and checks the
   samples, the ACR and the InfoFrames.
4. The shell: page `0x1000_7000`, a PLIC source, the frames into `hdmi_tx`; the HDMI DTS node.
5. Board: `aplay` of each of the twelve configurations on the HDMI board's monitor.

## 5. Progress

- Step 1 (082fc973): virtio_mmio holds four queues as arrays; virtio_input and the Linux bench's
  block device use the buses.
- Step 2 (0574f56c): virtio_snd; tb_virtio_snd drives the twelve configurations frame-exact, the
  control requests, STOP's wait and RELEASE's return. The bench plays a frame every 256 cycles: at 16
  it starved between its 23-frame buffers (the device then plays silence, as it should).
- Steps 3 and 4: hdmi_symbols (the pure-RTL symbol generator hdmi_tx now instantiates) places
  everything from the line's own edges, not a mode's numbers: the islands 4 pixels after HSYNC's
  leading edge (its active level learned from the level it holds during video), only while the last
  line had 100 pixels from that edge to video; the video preamble from the stream seen 12 pixels
  ahead. CTS is measured over every 49th frame's arrival. tb_hdmi_audio decodes the symbols with an
  independent sink model (192 islands in 12 frames of a small negative-sync mode, every audio frame
  in order with parity and block flags, ACR, both InfoFrames once a frame). The shell: page 0x07,
  PLIC source 13 (rv_soc_top's virtio_snd_irq), a keyboard+sound AXI arbiter (ID bit 2) into the
  VGA arbiter's second port, aud_valid/aud_frame out to the HDMI board's hdmi_tx; the HDMI board's
  DTB lists the device (tools/check-dts-timebase.py's @SOUND@).
