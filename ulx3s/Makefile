prog: ulx3s.bit
	fujprog ulx3s.bit

ulx3s.bit: ulx3s_out.config
	ecppack ulx3s_out.config ulx3s.bit

ulx3s_out.config: ulx3s.json
	nextpnr-ecp5 --85k --json ulx3s.json \
		--package CABGA381 \
		--lpf ulx3s_v20.lpf \
		--textcfg ulx3s_out.config

ulx3s.json: ulx3s.ys ulx3s.v ../smolrv64.v ../rs232rx.v ../rs232tx.v ../mem.hex
	yosys --top top ulx3s.ys 
