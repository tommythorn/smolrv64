# Smol RV64

There are literally hundreds of RISC-V implementations already and
I've even written some of them so why write yet another one?

The purpose of this implementation is to be as simple as possible and
as fast to implement as possible, thus, the RTL will look terribly
inefficient and almost look like a software simulator.  That's on
purpose!  Performance is *not* the goal.

Initially this will be a single-cycle implementation, but we expect we
will have to migrate to a micro-sequenced implementation (FPGA rams
generally want to be fed with a register).
