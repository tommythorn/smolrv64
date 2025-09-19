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

ulx3s.json: ulx3s.ys ulx3s.v $(SRC)/smolrv64.v $(SRC)/rs232rx.v $(SRC)/rs232tx.v $(SRC)/mem.even $(SRC)/mem.odd
	yosys --top top ulx3s.ys 
