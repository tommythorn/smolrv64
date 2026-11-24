`default_nettype none

// PLIC (SiFive layout, base 0x0C00_0000) for the probe SoC; smolrv64_plic_arbiter.v is its
// priority scan. NSRC sources are implemented: a source at or above NSRC has its priority and
// enable hardwired to zero, as the PLIC specification allows, so it never pends.
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
// MMIO contract (matches clint.v in rv_soc_top): `we` is a 1-cycle write-accept strobe; `re`
// is a 1-cycle read strobe -- the CLAIM read has a side effect (clear pending, mark in
// service) so it must fire exactly once, hence a strobe rather than a level. `rdata` is
// registered and valid the cycle after `re`.
//
// Gateway: a source becomes pending while its level is asserted and it is not in service;
// claim clears its pending bit and marks it in service; complete rearms it. Source 0 unused.
module plic #(parameter NSRC = 16)    // a multiple of 16, at most 64
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
   reg [NSRC-1:0] pending, enabled, in_service;
   wire [5:0]     pidx = addr[7:2];                 // the priority register a write or read names
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
`ifdef SMOLRV64_IRQ_STIM
   // SIM-ONLY (see rv_soc_top's SMOLRV64_IRQ_STIM): source 10 counts as enabled at priority >= 1
   // whatever the kernel wrote (it masks a source by writing priority 0 and never requests the
   // UART line before the tty opens), so the spurious level becomes a stream of interrupts.
   wire [NSRC-1:0]   en_eff   = enabled   | {{(NSRC-11){1'b0}}, 11'h400};
   wire [3*NSRC-1:0] prio_eff = prio_flat | ({{(3*NSRC-1){1'b0}}, 1'b1} << (10*3));
`else
   wire [NSRC-1:0]   en_eff   = enabled;
   wire [3*NSRC-1:0] prio_eff = prio_flat;
`endif
   smolrv64_plic_arbiter #(.NSRC(NSRC)) arb
     (.clock(clk), .pending(pending), .enabled(en_eff),
      .priority_flat(prio_eff), .threshold(threshold),
      .best_irq(best_irq), .has_irq(has_irq));

   assign meip = has_irq;
   assign seip = has_irq;

   wire [NSRC-1:0] source_level = src & ~{{(NSRC-1){1'b0}}, 1'b1};   // source 0 unused
   wire        is_claim = (addr >= 24'h201004) && (addr <= 24'h201007);
   wire        do_claim = re & is_claim & has_irq;
   // a COMPLETE write (rearm gateway) for a nonzero source -- src-11's completes clear in_service[11]
   wire        complete_evt = we & is_claim & (wdata[5:0] != 6'd0) & (wdata[5:0] < NSRC);
   // dbg source index: 10 = UART THRE/RDA (the /init console-write wedge under
   // investigation); was 11 (virtio-blk) during the root-mount-hang hunt.
   assign      dbg = {complete_evt, do_claim, seip, in_service[10], pending[10], source_level[10], best_irq};

   // next pending = gateway set, minus the just-claimed source
   reg [NSRC-1:0] n_pending;
   always @* begin
      n_pending = pending | (source_level & ~in_service);
      if (do_claim) n_pending[best_irq[$clog2(NSRC)-1:0]] = 1'b0;
   end

   // the enable bitmap after a write: 8 bytes, or the 32-bit word addr[2] names
   wire [63:0] en_64 = {{(64-NSRC){1'b0}}, enabled};
   wire [63:0] en_w  = wmask[4] ? wdata : addr[2] ? {wdata[31:0], en_64[31:0]} : {en_64[63:32], wdata[31:0]};
   always @(posedge clk) begin
      if (reset) begin
         pending<=0; enabled<=0; in_service<=0; threshold<=0;
      end else begin
         pending <= n_pending;
         if (do_claim) in_service[best_irq[$clog2(NSRC)-1:0]] <= 1'b1;
         if (we) begin
            if (addr <= 24'h0000FF) begin
               if (pidx < NSRC) prio[pidx[$clog2(NSRC)-1:0]] <= wdata[2:0];
            end else if (addr >= 24'h002080 && addr <= 24'h002087)
               enabled <= en_w[NSRC-1:0];
            else if (addr >= 24'h201000 && addr <= 24'h201003)
               threshold <= wdata[2:0];
            else if (complete_evt)                                  // complete: rearm gateway
               in_service[wdata[$clog2(NSRC)-1:0]] <= 1'b0;
         end
      end
   end

`ifdef PLIC_TRACE
   // Claim/complete pairing trace + the LOST-INTERRUPT detector.
   //
   // The gateway is `n_pending = pending | (source_level & ~in_service)`. So a source
   // left in_service -- claimed but never completed, or completed with a different id --
   // can NEVER re-pend: every later interrupt from it is dropped on the floor and the
   // waiter sleeps forever while the kernel idles. That is the shape of the wedge, so
   // detect it directly rather than inferring it from the console.
   always @(posedge clk) if (!reset) begin
      if (do_claim)
         $display("[PLIC c=%0t CLAIM irq=%0d pend=%h insvc=%h lvl=%h]",
                  $time/10, best_irq, pending, in_service, source_level);
      if (we && is_claim)
         $display("[PLIC c=%0t COMPL irq=%0d pend=%h insvc=%h lvl=%h]%s",
                  $time/10, wdata[5:0], pending, in_service, source_level,
                  (wdata[5:0] == 6'd0) ? "  <-- id 0: clears NOTHING" : "");
   end

   integer si;
   reg [31:0] stuck [0:NSRC-1];
   initial for (si = 0; si < NSRC; si = si + 1) stuck[si] = 0;
   always @(posedge clk) if (!reset)
      for (si = 1; si < NSRC; si = si + 1)
         if (in_service[si] & source_level[si]) begin
            stuck[si] <= stuck[si] + 1;
            if (stuck[si] == 32'd2000000)
               $display("[PLIC LOST c=%0t irq=%0d asserting its level while held in_service for 2M cycles -- it can no longer re-pend; every further interrupt from it is dropped (insvc=%h lvl=%h en=%h)]",
                        $time/10, si, in_service, source_level, enabled);
         end else stuck[si] <= 0;
`endif

   // the arbiter names only implemented sources
   always @(posedge clk) if (!reset && has_irq && best_irq >= NSRC)
      $fatal(1, "plic: the arbiter picked source %0d of %0d", best_irq, NSRC);

   // registered read (valid the cycle after `re`); claim returns best_irq
   always @(posedge clk) begin
      if (re) begin
         if (addr <= 24'h0000FF)                            rdata <= (pidx < NSRC) ? {61'd0, prio[pidx[$clog2(NSRC)-1:0]]} : 64'd0;
         else if (addr >= 24'h001000 && addr <= 24'h00107F) rdata <= {{(64-NSRC){1'b0}}, pending} >> ({addr[6:0]} * 8);
         else if (addr >= 24'h002080 && addr <= 24'h002087) rdata <= en_64;
         else if (addr >= 24'h201000 && addr <= 24'h201003) rdata <= {61'd0, threshold};
         else if (is_claim)                                 rdata <= {58'd0, best_irq};
         else                                               rdata <= 64'd0;
      end
   end
endmodule

`default_nettype wire
