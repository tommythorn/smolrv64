`default_nettype none

// PLIC (SiFive layout, base 0x0C00_0000) for the probe SoC -- a standalone extraction of
// the model embedded in smolrv64.v, reusing smolrv64_plic_arbiter.v for the priority scan.
// Only the S-mode context (context 1) is implemented (Linux runs external IRQs through it),
// and meip/seip are both driven by that single context's decision -- matching smolrv64.
//
// Register map (offset within the region, byte address):
//   0x000000..0x0000FF  priority[source]      (source = off[7:2], 3 bits each)   R/W
//   0x001000..0x00107F  pending bitmap                                          R (level)
//   0x002080..0x002087  enable bitmap (context 1)                              R/W
//   0x201000..0x201003  priority threshold (context 1)                         R/W
//   0x201004..0x201007  claim (read) / complete (write), context 1
//
// MMIO contract (matches clint.v in soc_top): `we` is a 1-cycle write-accept strobe; `re`
// is a 1-cycle read strobe -- the CLAIM read has a side effect (clear pending, mark in
// service) so it must fire exactly once, hence a strobe rather than a level. `rdata` is
// registered and valid the cycle after `re`.
//
// Gateway: a source becomes pending while its level is asserted and it is not in service;
// claim clears its pending bit and marks it in service; complete rearms it. Source 0 unused.
module plic #(parameter NSRC = 64)
  (input  wire             clk,
   input  wire             reset,
   input  wire             we,            // write accept (1-cycle)
   input  wire             re,            // read strobe (1-cycle; claim side effect fires here)
   input  wire [23:0]      addr,          // byte offset within the PLIC region
   input  wire [63:0]      wdata,
   input  wire [7:0]       wmask,
   output reg  [63:0]      rdata,
   input  wire [NSRC-1:0]  src,           // level-triggered sources (bit i; bit 0 unused)
   output wire             meip,
   output wire             seip,
   // debug tap for the interrupt-path ILA (src 11 = virtio_blk lifecycle):
   //   {complete_evt, do_claim, seip, in_service[11], pending[11], source_level[11], best_irq[5:0]}
   output wire [11:0]      dbg);

   reg [2:0]  prio [0:NSRC-1];
   reg [63:0] pending, enabled, in_service;
   reg [2:0]  threshold;
   integer i;
   initial begin
      pending=0; enabled=0; in_service=0; threshold=0;
      for (i=0;i<NSRC;i=i+1) prio[i]=0;
      rdata=0;
   end

   // flatten per-source priority for the arbiter port (source i -> [i*3 +: 3])
   wire [3*NSRC-1:0] prio_flat;
   genvar g;
   generate for (g=0; g<NSRC; g=g+1) assign prio_flat[g*3 +: 3] = prio[g]; endgenerate

   wire [5:0] best_irq;
   wire       has_irq;
   smolrv64_plic_arbiter arb
     (.clock(clk), .pending(pending), .enabled(enabled),
      .priority_flat(prio_flat), .threshold(threshold),
      .best_irq(best_irq), .has_irq(has_irq));

   assign meip = has_irq;
   assign seip = has_irq;

   wire [63:0] source_level = {{(64-NSRC){1'b0}}, src} & ~64'd1;   // source 0 unused
   wire        is_claim = (addr >= 24'h201004) && (addr <= 24'h201007);
   wire        do_claim = re & is_claim & has_irq;
   // a COMPLETE write (rearm gateway) for a nonzero source -- src-11's completes clear in_service[11]
   wire        complete_evt = we & is_claim & (wdata[5:0] != 6'd0);
   // dbg source index: 10 = UART THRE/RDA (the /init console-write wedge under
   // investigation); was 11 (virtio-blk) during the root-mount-hang hunt.
   assign      dbg = {complete_evt, do_claim, seip, in_service[10], pending[10], source_level[10], best_irq};

   // next pending = gateway set, minus the just-claimed source
   reg [63:0] n_pending;
   always @* begin
      n_pending = pending | (source_level & ~in_service);
      if (do_claim) n_pending[best_irq] = 1'b0;
   end

   always @(posedge clk) begin
      if (reset) begin
         pending<=0; enabled<=0; in_service<=0; threshold<=0;
      end else begin
         pending <= n_pending;
         if (do_claim) in_service[best_irq] <= 1'b1;
         if (we) begin
            if (addr <= 24'h0000FF)
               prio[addr[7:2]] <= wdata[2:0];
            else if (addr >= 24'h002080 && addr <= 24'h002087) begin
               if (wmask[4])      enabled        <= wdata;          // 8-byte write
               else if (addr[2])  enabled[63:32] <= wdata[31:0];
               else               enabled[31:0]  <= wdata[31:0];
            end else if (addr >= 24'h201000 && addr <= 24'h201003)
               threshold <= wdata[2:0];
            else if (is_claim) begin                                // complete: rearm gateway
               if (wdata[5:0] != 0) in_service[wdata[5:0]] <= 1'b0;
            end
         end
      end
   end

   // registered read (valid the cycle after `re`); claim returns best_irq
   always @(posedge clk) begin
      if (re) begin
         if (addr <= 24'h0000FF)                            rdata <= {61'd0, prio[addr[7:2]]};
         else if (addr >= 24'h001000 && addr <= 24'h00107F) rdata <= pending >> ({addr[6:0]} * 8);
         else if (addr >= 24'h002080 && addr <= 24'h002087) rdata <= enabled;
         else if (addr >= 24'h201000 && addr <= 24'h201003) rdata <= {61'd0, threshold};
         else if (is_claim)                                 rdata <= {58'd0, best_irq};
         else                                               rdata <= 64'd0;
      end
   end
endmodule

`default_nettype wire
