`default_nettype none
// Timing spike (Stage 4, docs/PLAN-2026-09-24-frontend-stage4.md): the fetch ring's consumption
// loop. The registered head pointer selects HW halfwords from an RS-slot ring (the rotation), the
// real aligner carves up to IW instructions, and what it consumed advances the head, the head's PC
// and sequence number -- all in one cycle. Appends at the tail depend only on the tail and count
// registers, so they are off this loop; they are modelled so the ring's write ports exist. The
// aligned bundle is registered, standing in for the instruction register. Compare against
// `make ooc MODULE=fetch GENERICS="IW=3 HW=8"`, today's loop (pc_q -> window -> aligner -> pc_q).
//   make ooc MODULE=ooo2_ring_spike GENERICS="RS=16"     (32-byte ring; RS=32 for 64 bytes)
module ooo2_ring_spike #(parameter IW=3, HW=8, RS=16, PCW=64, SEQW=8)
   (input  wire clk, input wire reset,
    input  wire [HW*16-1:0] app_in,            // the I$ read's halfwords, fetch order
    input  wire [3:0]       app_n_in,          // how many of them to append (0..HW)
    input  wire             take_in,           // decode takes the bundle this cycle
    output reg  [PCW-1:0]   out);
   localparam RB  = $clog2(RS);
   localparam PBW = $clog2(HW+2);
   // flop-wrapped stimulus
   reg [HW*16-1:0] app_q;  reg [3:0] app_n_q;  reg take_q;
   always @(posedge clk) begin app_q <= app_in; app_n_q <= app_n_in; take_q <= take_in; end

   reg [15:0]     ring [0:RS-1];
   reg [RB-1:0]   head, tail;
   reg [RB:0]     cnt;
   reg [PCW-1:0]  hpc;
   reg [SEQW-1:0] hseq;

   // the head window: HW halfwords rotated from the registered head (RB-bit wrap)
   wire [HW*16-1:0] win;
   genvar k;
   generate for (k = 0; k < HW; k = k + 1) begin : w
      wire [RB-1:0] ix = head + k[RB-1:0];
      assign win[k*16 +: 16] = ring[ix];
   end endgenerate
   wire [PBW-1:0] avail = (cnt >= HW[RB:0]) ? HW[PBW-1:0] : cnt[PBW-1:0];

   wire [IW-1:0]      al_v;   wire [IW*32-1:0] al_inst;  wire [IW*PCW-1:0] al_pc;
   wire [IW*PBW-1:0]  al_offs;  wire [IW*SEQW-1:0] al_seq;  wire [PBW-1:0] al_cons;  wire al_brt;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_al
     (.hwin(win), .avail(avail), .base_pc(hpc), .base_seq(hseq), .solo_all(1'b0), .bytes_late(1'b0),
      .valid(al_v), .inst(al_inst), .pc(al_pc), .offs(al_offs), .seq(al_seq),
      .consumed(al_cons), .br_term(al_brt));

   wire [PBW-1:0] cons  = take_q ? al_cons : {PBW{1'b0}};
   wire [RB-1:0]  cons_r = {{(RB-PBW+1){1'b0}}, cons[PBW-2:0]};   // consumed <= HW < RS
   wire [1:0]     ninst = {1'b0, al_v[0]} + {1'b0, al_v[1]} + {1'b0, (IW > 2) ? al_v[IW-1] : 1'b0};
   wire           room  = (cnt <= RS[RB:0] - HW[RB:0]);
   wire [3:0]     app_n = room ? app_n_q : 4'd0;
   wire [RB:0]    app_w = {{(RB-3){1'b0}}, app_n};
   integer j;
   always @(posedge clk) begin
      if (reset) begin
         head <= {RB{1'b0}}; tail <= {RB{1'b0}}; cnt <= {(RB+1){1'b0}};
         hpc <= {PCW{1'b0}}; hseq <= {SEQW{1'b0}};
      end else begin
         head <= head + cons_r;
         hpc  <= hpc + {{(PCW-PBW-1){1'b0}}, cons, 1'b0};
         hseq <= hseq + (take_q ? {{(SEQW-2){1'b0}}, ninst} : {SEQW{1'b0}});
         tail <= tail + app_w[RB-1:0];
         cnt  <= cnt + app_w - {{(RB+1-PBW){1'b0}}, cons};
         for (j = 0; j < HW; j = j + 1)
            if (j < app_n) ring[tail + j[RB-1:0]] <= app_q[j*16 +: 16];
      end
   end

   // the IR: the aligned bundle, reduced to one register so nothing is optimized away
   wire [PCW-1:0] red = al_pc[0 +: PCW] ^ {32'd0, al_inst[0 +: 32]} ^ {32'd0, al_inst[(IW-1)*32 +: 32]}
                      ^ {{(PCW-IW*PBW){1'b0}}, al_offs} ^ {{(PCW-IW*SEQW){1'b0}}, al_seq}
                      ^ {{(PCW-IW-1){1'b0}}, al_v, al_brt};
   always @(posedge clk) out <= reset ? {PCW{1'b0}} : red;
endmodule
`default_nettype wire
