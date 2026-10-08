// hid-bridge -- USB keyboards and mice to SmolRV64's virtio_input, over one wire.
//
// ps2x2pico (github.com/No0ne/ps2x2pico, MIT) with its PS/2 side replaced. usbin.c is its USB
// host side, kept: TinyUSB with hubs, report and boot protocols, NKRO keyboards, any number of
// keyboards and mice at once. This file turns what it reports into Linux input events and sends
// each on UART1 TX (GPIO4) at 1 Mbps, 8N1, to virtio_input's HID line (src/virtio_input.v):
//
//   event  {type[1:0], code[9:0], value[15:0]}, the value sign-extended to 32 bits by the FPGA
//   bytes  1 type[1:0] code[9:5] | 0 code[4:0] value[15:14] | 0 value[13:7] | 0 value[6:0]
//
// Bit 7 marks an event's first byte, so the receiver resynchronizes after a lost byte. Every
// USB report ends with SYN_REPORT. Keys go through Linux's own HID table, so the guest sees
// what a USB keyboard on Linux would send; autorepeat is the guest's (EV_REP). The lock-key
// LEDs are kept here: the line has no return path for the guest's LED state.
//
// UART0 (GPIO0, 115200) carries ps2x2pico's log of attached devices.
#include "hid-bridge.h"
#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

#define EV_UART uart1
#define EV_TX   4
#define EV_BAUD 1000000          // virtio_input's HID_BAUD

#define EV_SYN    0
#define EV_KEY    1
#define EV_REL    2
#define BTN_LEFT  0x110
#define REL_X     0
#define REL_Y     1
#define REL_WHEEL 8

// HID keyboard usage -> Linux key code: hid_keyboard[] of Linux's drivers/hid/hid-input.c,
// with KEY_UNKNOWN as 0 (not sent).
static const u8 hid_keyboard[256] = {
    0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
   50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
    4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
   27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
   65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
  105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
   72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
  191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
  115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
  122,123, 90, 91, 85,  0,  0,  0,  0,  0,  0,  0,111,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,179,180,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,111,  0,  0,  0,  0,  0,  0,  0,
   29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
  150,158,159,128,136,177,178,176,142,152,173,140,  0,  0,  0,  0
};

static bool unsynced;            // events sent since the last SYN_REPORT

static void ev(u8 type, u16 code, s32 value) {
  if (value > 32767) value = 32767;
  if (value < -32768) value = -32768;
  u32 p = (u32)(type & 3) << 26 | (u32)(code & 0x3ff) << 16 | (u16)value;
  u8 b[4] = {0x80 | (p >> 21 & 0x7f), p >> 14 & 0x7f, p >> 7 & 0x7f, p & 0x7f};
  uart_write_blocking(EV_UART, b, sizeof b);
  unsynced = true;
}

void ev_syn(void) {
  if (unsynced) ev(EV_SYN, 0, 0);
  unsynced = false;
}

static u8 leds;
static bool leds_changed;

void kb_send_key(u8 key, bool is_key_pressed, u8 modifiers) {
  (void)modifiers;
  if (hid_keyboard[key]) ev(EV_KEY, hid_keyboard[key], is_key_pressed);
  if (is_key_pressed) {
    u8 led = key == HID_KEY_NUM_LOCK    ? KEYBOARD_LED_NUMLOCK
           : key == HID_KEY_CAPS_LOCK   ? KEYBOARD_LED_CAPSLOCK
           : key == HID_KEY_SCROLL_LOCK ? KEYBOARD_LED_SCROLLLOCK : 0;
    leds ^= led;
    leds_changed |= led != 0;
  }
}

static u8 ms_buttons;

void ms_send_movement(u8 buttons, s16 x, s16 y, s16 z) {
  for (int i = 0; i < 5; i++)    // left, right, middle, side, extra: BTN_LEFT..BTN_EXTRA
    if ((buttons ^ ms_buttons) >> i & 1) ev(EV_KEY, BTN_LEFT + i, buttons >> i & 1);
  ms_buttons = buttons;
  if (x) ev(EV_REL, REL_X, x);
  if (y) ev(EV_REL, REL_Y, y);
  if (z) ev(EV_REL, REL_WHEEL, z);
}

int main() {
  board_init();
  uart_init(EV_UART, EV_BAUD);
  gpio_set_function(EV_TX, GPIO_FUNC_UART);
  printf("\n%s-%s\n", PICO_PROGRAM_NAME, PICO_PROGRAM_VERSION_STRING);

  tuh_hid_set_default_protocol(HID_PROTOCOL_REPORT);
  tusb_init();

  while (1) {
    tuh_task();
    if (leds_changed) {          // a control transfer: from the main loop, not a report callback
      leds_changed = false;
      tuh_kb_set_leds(leds);
    }
  }
}

void reset() {
  printf("\n\n *** PANIC via tinyusb: watchdog reset!\n\n");
  watchdog_enable(100, false);
}
