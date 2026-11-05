# The boards on coffee, for tools/gate.sh and tools/board-gate.sh: source it with BOARD set (2g, 4g,
# hdmi, or empty for the one RK board a host has) and MAIN the main checkout. It sets:
#   PLAT    the platform directory (relative to the repository) and PROJ its project
#   SCREEN  the screen session on the board's console, CON its log, TTYDEV and BAUD the console
#   JTAG    the board's cable (make program / flash), DTB the device tree it boots, DTBA where
#   BMAC    the MAC its FPGA's DNA gives it, which its boot must show
# It returns 1 for a board it does not know.
case "${BOARD:-}" in
   ""|2g|4g) PLAT=platforms/rk-xcku5p-f-v1.2; PROJ=rk_xcku5p; DTBA=fefff000; BAUD=3000000 ;;&
   "")   BMAC="" ;;
   2g)   SCREEN=b2g;   JTAG=0ABC02A;      DTB=ubuntu-nfs.dtb;      BMAC=02:53:7c:76:21:25 ;;
   4g)   SCREEN=b4g;   JTAG=0ABC01A;      DTB=ubuntu-nfs-4g.dtb;   BMAC=02:53:64:12:43:42 ;;
   hdmi) PLAT=platforms/hdmi-xcku5p; PROJ=hdmi_xcku5p; DTBA=befff000; BAUD=2000000
         SCREEN=bhdmi; JTAG=210251A08870; DTB=ubuntu-nfs-hdmi.dtb; BMAC=02:53:4c:38:41:00
         # its console is a CP2105's interface 0, beside the JTAG on the one USB-C connector
         TTYDEV=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_0066394E-if00-port0 ;;
   *)    return 1 ;;
esac
# an RK board's console is its JTAG cable's second channel (the cable's serial less its 'A')
[ -n "${JTAG:-}" ] && [ -z "${TTYDEV:-}" ] && TTYDEV=/dev/serial/by-id/usb-Xilinx_my_product_desc_${JTAG%A}-if01-port0
[ -n "${BOARD:-}" ] && CON=$MAIN/workloads/ubuntu/screenlog.$BOARD
return 0
