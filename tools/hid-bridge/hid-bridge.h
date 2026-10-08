// hid-bridge: what usbin.c (the USB host side, from ps2x2pico) calls in hid-bridge.c.
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "tusb.h"

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

void kb_send_key(u8 key, bool is_key_pressed, u8 modifiers);   // key: a HID keyboard usage
void ms_send_movement(u8 buttons, s16 x, s16 y, s16 z);       // buttons: bit i is button i+1
void ev_syn(void);                                             // ends the events of one report
void tuh_kb_set_leds(u8 leds);
