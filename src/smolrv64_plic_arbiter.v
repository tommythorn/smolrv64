// PLIC priority arbiter: pick the highest-priority pending+enabled source
// whose priority exceeds the threshold. Two-cycle pipeline (scan 4 groups of
// 16, then merge) to keep the path short. Pure scan over the register state,
// which the smolrv64 core continues to own; this module holds no architectural
// state, only the pipeline registers.
//
// priority_flat packs the 64 per-source 3-bit priorities: source i is at
// priority_flat[i*3 +: 3] (source 0 is unused, per the PLIC spec).
module smolrv64_plic_arbiter
  (input  wire         clock,
   input  wire [ 63:0] pending,
   input  wire [ 63:0] enabled,
   input  wire [191:0] priority_flat,
   input  wire [  2:0] threshold,
   output reg  [  5:0] best_irq = 0,
   output reg          has_irq  = 0);

   // Stage 1: scan 4 groups of 16, register the per-group winner.
   reg [5:0] grp_irq_r [0:3];
   reg [2:0] grp_pri_r [0:3];
   integer   i, g;
   always @(posedge clock) begin : stage1
      reg [5:0] gi;
      reg [2:0] gp;
      for (g = 0; g < 4; g = g + 1) begin
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

   // Stage 2: merge the 4 registered group results.
   always @(posedge clock) begin : stage2
      reg [5:0] b_irq;
      reg [2:0] b_pri;
      b_irq = grp_irq_r[0]; b_pri = grp_pri_r[0];
      if (grp_pri_r[1] > b_pri) begin b_irq = grp_irq_r[1]; b_pri = grp_pri_r[1]; end
      if (grp_pri_r[2] > b_pri) begin b_irq = grp_irq_r[2]; b_pri = grp_pri_r[2]; end
      if (grp_pri_r[3] > b_pri) begin b_irq = grp_irq_r[3]; b_pri = grp_pri_r[3]; end
      best_irq <= b_irq;
      has_irq  <= b_irq != 0;
   end
endmodule
