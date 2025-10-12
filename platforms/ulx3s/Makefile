SRC=../../src

prog: ulx3s.bit
	fujprog ulx3s.bit

ulx3s.bit: ulx3s_out.config
	ecppack ulx3s_out.config ulx3s.bit

ulx3s_out.config: ulx3s.json
	nextpnr-ecp5 --85k --json ulx3s.json \
		--package CABGA381 \
		--lpf ulx3s_v20.lpf \
		--textcfg ulx3s_out.config

ulx3s.json: ulx3s.ys ulx3s.v $(SRC)/smolrv64.v $(SRC)/uart5.v $(SRC)/mem.even $(SRC)/mem.odd
	yosys --top top ulx3s.ys 

sim_top.vcd: ulx3s_tb.v ulx3s.v $(SRC)/smolrv64.v $(SRC)/uart5.v $(SRC)/mem.even $(SRC)/mem.odd
	iverilog -o sim_top -I$(SRC) ulx3s_tb.v ulx3s.v $(SRC)/smolrv64.v $(SRC)/uart5.v
	vvp sim_top +even=mem.even +odd=mem.odd

sim_top_verbose:
	iverilog -o sim_top_verbose -DSIMULATE -DDISASM -DNO_TIMEOUT -I../../src ulx3s_tb.v ulx3s.v ../../src/smolrv64.v ../../src/uart5.v
	vvp sim_top_verbose +even=mem.even +odd=mem.odd
