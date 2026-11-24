// PLIC priority arbiter: pick the highest-priority pending+enabled source
// whose priority exceeds the threshold. Two-cycle pipeline (scan groups of 16,
// then merge) to keep the path short. Pure scan over the register state; this
// module holds no architectural state, only the pipeline registers.
//
// priority_flat packs the NSRC per-source 3-bit priorities: source i is at
// priority_flat[i*3 +: 3] (source 0 is unused, per the PLIC spec).
module smolrv64_plic_arbiter #(parameter NSRC = 16)     // a multiple of 16, at most 64
  (input  wire              clock,
   input  wire [NSRC-1:0]   pending,
   input  wire [NSRC-1:0]   enabled,
   input  wire [3*NSRC-1:0] priority_flat,
   input  wire [  2:0]      threshold,
   output reg  [  5:0]      best_irq = 0,
   output reg               has_irq  = 0);

   localparam integer NG = NSRC / 16;
   // Stage 1: scan each group of 16, register the per-group winner.
   reg [5:0] grp_irq_r [0:NG-1];
   reg [2:0] grp_pri_r [0:NG-1];
   integer   i, g;
   always @(posedge clock) begin : stage1
      reg [5:0] gi;
      reg [2:0] gp;
      for (g = 0; g < NG; g = g + 1) begin
         gi = 0; gp = 0;
         for (i = g * 16; i < (g + 1) * 16; i = i + 1)
            if (i > 0 &&
                pending[i] && enabled[i] &&
                priority_flat[i*3 +: 3] > threshold &&
                priority_flat[i*3 +: 3] > gp) begin
               gi = i[5:0];
               gp = priority_flat[i*3 +: 3];
            end
         grp_irq_r[g] <= gi;
         grp_pri_r[g] <= gp;
      end
   end

   // Stage 2: merge the registered group results, the lowest group first on a tie.
   always @(posedge clock) begin : stage2
      reg [5:0] b_irq;
      reg [2:0] b_pri;
      b_irq = grp_irq_r[0]; b_pri = grp_pri_r[0];
      for (g = 1; g < NG; g = g + 1)
         if (grp_pri_r[g] > b_pri) begin b_irq = grp_irq_r[g]; b_pri = grp_pri_r[g]; end
      best_irq <= b_irq;
      has_irq  <= b_irq != 0;
   end
endmodule
