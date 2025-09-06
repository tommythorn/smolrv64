# Smol RV64

There are literally hundreds of RISC-V implementations already and
I've even written some of them so why write yet another one?

The goal of this implementation is to reach functionality as quickly
as possible, thus, the RTL will prioritize simplicity over efficientcy
and will resemble a software simulator.  That's on purpose!
Performance is *not* the goal (initially)

# Status

RV64IMC is fully implemented except for some system features and all
debug.

Currently two dev boards are directly supported: ULX3S and RX-XCKU5P-F.

## Performace

Performance isn't a priority, but IPC is about 0.22 at 25 MHz (ULX3S) and 200 MHz (RX-XCKU5P).

# Coming soon

Atomic support is next

# Coming eventually

- Full system support, including virtual memory
- Caches

Floating point (F and G) support remains
