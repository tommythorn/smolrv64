`timescale 1ns/1ps
`default_nettype none
// THE ORACLE FRONTEND -- a bench instrument, built only with -DSMOLRV64_ORACLE_FE (the ideal-frontend
// study, workloads/rvbench `make ideal`). It is the bundle register's source in place of fetch: the
// correct path a previous lockstep recorded (+trace_out: one retired PC a line), read from
// +oracle=<file>, each instruction's bits from the program image (+fw, loaded at BASE). Up to FW
// instructions a cycle, each slot with its own next PC, so taken branches sit inside a bundle and
// nothing mispredicts; no I$, no ring, no alignment. A SYSTEM or FENCE op travels alone, as the
// aligner sends it. A redirect that still comes (a replay, a CSR's serialisation) restarts at the
// trace position its sequence number was given, or, for a replay -- the op again, under the next
// number (smolrv64_core redirect_seq) -- the number before's. A redirect the trace does not hold --
// the backend's own speculation (a branch resolved on a load that read past an older store), or the
// run leaving the recorded path, as the printout of its own timing does -- sends the halfwords after
// it, predicting nothing, until a redirect lands on a recorded position again. +oracle_cut ends a
// bundle at its first taken branch, as the aligner does: the ideal less one taken branch a cycle.
// Guarded by the same define as its one instance (smolrv64_frontend): every other build, and
// every synthesis flow, sees an empty file.
`ifdef SMOLRV64_ORACLE_FE
module smolrv64_oracle_fe
  #(parameter integer FW   = 6,
    parameter integer PCW  = 40,
    parameter integer SEQW = 9,
    parameter [63:0]  BASE = 64'h8000_0000,
    parameter integer IMGB = 22,            // the image: up to 4 MiB of code from BASE
    parameter integer TRB  = 24)            // the trace: up to 16 M retired instructions
   (input  wire                 clk,
    input  wire                 reset,
    input  wire                 redirect,
    input  wire [PCW-1:0]       redirect_pc,
    input  wire [SEQW-1:0]      redirect_seq,
    input  wire                 irq_inject,
    input  wire                 ready,           // the bundle register takes what is presented
    output reg                  valid,
    output reg  [FW-1:0]        sv,
    output reg  [FW*32-1:0]     inst,
    output reg  [FW*PCW-1:0]    pc,
    output reg  [FW*SEQW-1:0]   seq,
    output reg  [FW-1:0]        tk,              // slot j's next PC is a target...
    output reg  [FW*PCW-1:0]    tgt,             // ...this one
    output wire [SEQW-1:0]      cur_seq);

   localparam [PCW-1:0] ENDM = {PCW{1'b1}};     // past the trace's last instruction
   reg [7:0]      img  [0:(1 << IMGB) - 1];
   reg [PCW-1:0]  tr   [0:(1 << TRB) - 1];
   reg [TRB:0]    smap [0:(1 << SEQW) - 1];      // each sequence number's trace position, and whether it had one
   reg [TRB-1:0]  k;                            // the next position to send...
   reg [SEQW-1:0] sq;                           // ...and its sequence number
   reg            off;                          // off the recorded path: sequential from opc
   reg [PCW-1:0]  opc;
   assign cur_seq = sq;

   integer i, n, fd;
   reg [8*256-1:0] fname;
   reg             cut;                         // +oracle_cut: a bundle ends at a taken branch
   initial begin
      cut = $test$plusargs("oracle_cut");
      for (i = 0; i < (1 << TRB); i = i + 1) tr[i] = ENDM;
      for (i = 0; i < (1 << IMGB); i = i + 1) img[i] = 8'd0;
      if (!$value$plusargs("oracle=%s", fname))
         $fatal(1, "smolrv64_oracle_fe: +oracle=<file>, the retired PCs of a +trace_out run");
      $readmemh(fname, tr);
      if (!$value$plusargs("fw=%s", fname)) $fatal(1, "smolrv64_oracle_fe: +fw=<image>");
      fd = $fopen(fname, "rb");
      if (fd == 0) $fatal(1, "smolrv64_oracle_fe: cannot open %0s", fname);
      n = $fread(img, fd);
      $fclose(fd);
      if (n >= (1 << IMGB)) $fatal(1, "smolrv64_oracle_fe: the image fills its %0d bytes", 1 << IMGB);
      for (i = 0; i < (1 << TRB) && tr[i] != ENDM; i = i + 1) ;
      $display("smolrv64_oracle_fe: %0d bytes of code at %h, a recorded path of %0d instructions%0s", n, BASE, i,
               cut ? ", bundles cut at a taken branch" : "");
      k = 0;  sq = 0;  off = 1'b0;  opc = {PCW{1'b0}};
      for (i = 0; i < (1 << SEQW); i = i + 1) smap[i] = {(TRB+1){1'b0}};
   end

   // The bundle: slot j is the trace's position k + j (off the path: the halfwords after opc), its
   // next PC the position after it; a SYSTEM or FENCE op is a bundle of its own.
   reg [PCW-1:0] p, nx;
   reg [31:0]    w;
   reg [63:0]    o;                              // p's offset in the image
   reg           stop, sys;
   reg [3:0]     nv;                             // slots presented
   integer j;
   always @* begin
      sv = {FW{1'b0}};  tk = {FW{1'b0}};  inst = {(FW*32){1'b0}};  pc = {(FW*PCW){1'b0}};
      seq = {(FW*SEQW){1'b0}};  tgt = {(FW*PCW){1'b0}};
      stop = 1'b0;  nv = 4'd0;  p = off ? opc : tr[k];
      for (j = 0; j < FW; j = j + 1) begin
         if (!off) p = tr[k + j[TRB-1:0]];
         o   = {{(64-PCW){1'b0}}, p} - BASE;
         w   = (o + 3 < (64'd1 << IMGB)) ? {img[o[IMGB-1:0] + 3], img[o[IMGB-1:0] + 2],
                                            img[o[IMGB-1:0] + 1], img[o[IMGB-1:0]]} : 32'd0;
         sys = (w[1:0] == 2'b11) & ((w[6:0] == 7'h73) | (w[6:0] == 7'h0f));
         if (!stop && p != ENDM && !(sys && j != 0)) begin
            nx = off ? p + ((w[1:0] == 2'b11) ? 4 : 2) : tr[k + j[TRB-1:0] + 1];
            if (nx == ENDM) nx = p + ((w[1:0] == 2'b11) ? 4 : 2);
            sv[j] = 1'b1;
            inst[j*32 +: 32]     = w;
            pc[j*PCW +: PCW]     = p;
            seq[j*SEQW +: SEQW]  = sq + j[SEQW-1:0];
            tk[j]                = nx != p + ((w[1:0] == 2'b11) ? 4 : 2);
            tgt[j*PCW +: PCW]    = nx;
            nv = nv + 4'd1;
            if (off) p = nx;
            if (sys || (cut && tk[j])) stop = 1'b1;
         end else stop = 1'b1;
      end
      valid = sv[0];
   end

   reg ended;
   reg [63:0] n_left, n_back, n_offi;            // redirects off the path and back onto it; ops sent off it
   initial begin ended = 1'b0;  n_left = 0;  n_back = 0;  n_offi = 0; end
   wire [TRB:0] ea = smap[redirect_seq], eb = smap[redirect_seq - 1'b1];
   wire         a_ok = (redirect_seq != sq) & ea[TRB] & (tr[ea[TRB-1:0]] == redirect_pc);
   wire         b_ok = eb[TRB] & (tr[eb[TRB-1:0]] == redirect_pc);
   always @(posedge clk) begin
      if (!reset && irq_inject) $fatal(1, "smolrv64_oracle_fe: an interrupt to present; the oracle replays code that takes none");
      if (reset) begin
         k <= 0;  sq <= 0;  off <= 1'b0;
      end else if (redirect) begin
         // the restart's sequence number names its position: one sent before (the next after a
         // serialising op), the next to send, or -- a replay, the op again under the next number --
         // the number before's
         sq <= redirect_seq;
         if (a_ok)                                    begin k <= ea[TRB-1:0];  off <= 1'b0; end
         else if (!off && redirect_seq == sq && tr[k] == redirect_pc) ;
         else if (b_ok)                               begin k <= eb[TRB-1:0];  off <= 1'b0; end
         else begin
            off <= 1'b1;  opc <= redirect_pc;
            if (!off) n_left <= n_left + 1;
         end
         if (off && (a_ok || b_ok)) n_back <= n_back + 1;
      end else if (ready && valid) begin
         for (j = 0; j < FW; j = j + 1)
            if (sv[j]) smap[sq + j[SEQW-1:0]] <= {~off, k + j[TRB-1:0]};
         sq <= sq + nv[SEQW-1:0];
         if (off) begin opc <= p;  n_offi <= n_offi + {60'd0, nv}; end
         else k <= k + {{(TRB-4){1'b0}}, nv};
      end
      if (!reset && !off && !ended && tr[k] == ENDM) begin
         ended <= 1'b1;
         $display("smolrv64_oracle_fe: the recorded path ends at position %0d", k);
      end
   end
   final $display("smolrv64_oracle_fe: %0d redirects left the recorded path, %0d came back to it; %0d instructions sent off it; at position %0d of the path",
                  n_left, n_back, n_offi, k);
endmodule
`endif
