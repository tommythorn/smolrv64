# The HDMI-XCKU5P board (1 GiB DDR4, HDMI out). The flow is platforms/xcku5p/board.mk's.
# Its console is the CP2105's interface 0, on the one USB-C connector beside the JTAG.
PROJECT    := hdmi_xcku5p
FLASH_PART := mt25qu128-spi-x1_x2_x4
FLASH_MB   := 16
MONITOR    := monitor-hdmi.bin
TTY        := /dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_0066394E-if00-port0
include ../xcku5p/board.mk
