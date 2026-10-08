# hid-bridge: USB keyboards and mice for SmolRV64

A Raspberry Pi Pico is the USB host for keyboards and mice, plugged in directly or through a
hub, with any number of each. It sends finished Linux input events to the FPGA over one UART
wire. There, `virtio_input` (`src/virtio_input.v`) delivers them to the guest as the virtio
keyboard and mouse at `0x1000_4000`.

The USB side is `usbin.c` from [ps2x2pico](https://github.com/No0ne/ps2x2pico) (MIT). It
handles report and boot protocols and NKRO keyboards. `hid-bridge.c` replaces ps2x2pico's PS/2
output with the event line.

## Wiring: three wires

| FPGA board, 40-pin header J1 | Pico |
|---|---|
| pin 1, GND | pin 38, GND |
| pin 2, +5V | pin 40, VBUS: powers the Pico and the USB devices on its port |
| pin 4, IO1_P (D11, LVCMOS33) | pin 6, GP4 (UART1 TX) |

The keyboard, mouse, or a hub goes on the Pico's micro-USB port through an OTG adapter.
UART0 (GP0, pin 1, 115200 baud) logs the devices as they attach.

## The line

The line runs at 1 Mbps, 8N1. Each event is 28 bits, `{type[1:0], code[9:0], value[15:0]}`,
with the value sign-extended to 32 bits. It is sent as four bytes of 7 bits each, most
significant first. Bit 7 is set on the first byte only, so the receiver resynchronizes on the
next event after a lost byte. Each USB report ends with `SYN_REPORT`.

- **Keys:** mapped through Linux's own HID table (`hid_keyboard[]` in `drivers/hid/hid-input.c`).
  The guest autorepeats.
- **Mouse:** `BTN_LEFT` through `BTN_EXTRA`, plus `REL_X`, `REL_Y` and `REL_WHEEL`.
- **Lock-key LEDs:** the Pico toggles them itself, because the line has no return path.

Diagnostics on the FPGA side: `0x1000_4F00[31:16]` counts events lost to a full FIFO, and
`0x1000_4F04[31:16]` counts framing errors.

## Build and flash

The firmware needs pico-sdk 1.5.1 with TinyUSB 0.17.0, the same versions ps2x2pico uses.

```sh
git clone --branch 1.5.1 https://github.com/raspberrypi/pico-sdk
git -C pico-sdk submodule update --init lib/tinyusb
git -C pico-sdk/lib/tinyusb fetch origin tag 0.17.0 && git -C pico-sdk/lib/tinyusb checkout 0.17.0
export PICO_SDK_PATH=$PWD/pico-sdk          # and arm-none-eabi-gcc on PATH
cmake -S tools/hid-bridge -B build/hid-bridge && make -C build/hid-bridge
```

To flash, hold BOOTSEL while plugging the Pico into a computer, then copy
`build/hid-bridge/hid-bridge.uf2` to the drive that appears.
