`default_nettype none

// Standalone Sv39 address-translation unit: a small TLB + a 3-level page-table
// walker. Algorithm + PTE format lifted from smolrv64.v (cache-entangled there;
// this is a clean cache-less version for the sharded-OoO probe). PTE layout:
//   V[0] R[1] W[2] X[3] U[4] G[5] A[6] D[7]  PPN=[53:10]  N(NAPOT)=[63]
// access: 0=fetch 1=load 2=store 3=amo.  satp[63:60]=MODE (0=Bare, 8=Sv39).
//
// Interface is COMBINATIONAL on the common path: when `req_valid`, a TLB hit (or
// Bare mode, or a non-canonical address) resolves THIS cycle -- `t_ready`=1 with
// `t_paddr`/`t_fault`/`t_cause` valid. A TLB *miss* under Sv39 starts a page-table
// walk: `t_ready`=0 (the caller holds the request and stalls) until the walk fills
// the TLB and pulses the registered result (`w_done`). The caller holds req_vaddr
// stable across a walk. PTEs are read through a registered memory port (drive
// ptw_addr/ptw_read, ptw_rdata valid next cycle when ptw_rvalid). Page-fault cause
// is the standard 12/13/15 (fetch/load/store; amo uses store).
module mmu
  #(parameter AW   = 56,           // physical address width produced
    parameter TLBN = 16,           // TLB entries (direct-mapped)
    parameter TLBI = 4)            // clog2(TLBN)
   (input  wire        clk,
    input  wire        reset,
    // request (combinational; held by the caller until t_ready)
    input  wire        req_valid,
    input  wire [63:0] req_vaddr,
    input  wire [1:0]  req_access,    // 0=fetch 1=load 2=store 3=amo
    input  wire [1:0]  priv,          // current (effective) privilege
    input  wire        sum,           // mstatus.SUM
    input  wire        mxr,           // mstatus.MXR
    input  wire [63:0] satp,
    input  wire        flush,         // sfence.vma: invalidate the TLB
    // PTW memory port (registered read: drive ptw_addr, ptw_rdata valid next cycle)
    output reg  [AW-1:0] ptw_addr,
    output reg         ptw_read,
    input  wire [63:0] ptw_rdata,
    input  wire        ptw_rvalid,
    // combinational translation result (valid this cycle when t_ready)
    output wire        t_ready,       // translation resolved this cycle (else: walking)
    output wire [AW-1:0] t_paddr,
    output wire        t_fault,
    output wire [3:0]  t_cause);      // 12=instr, 13=load, 15=store page fault

   wire        xlate = (satp[63:60] == 4'd8);   // 8 = Sv39, else Bare (identity)

   wire [8:0]  vpn2 = req_vaddr[38:30];
   wire [8:0]  vpn1 = req_vaddr[29:21];
   wire [8:0]  vpn0 = req_vaddr[20:12];
   wire        noncanon = (req_vaddr[63:39] != {25{req_vaddr[38]}});

   wire [3:0]  pf_cause = (req_access == 2'd0) ? 4'd12 :
                          (req_access == 2'd1) ? 4'd13 : 4'd15;

   // -------------------- TLB (direct-mapped on VPN[3:0] of vpn0) --------------------
   reg              tlb_v   [0:TLBN-1];
   reg [26:0]       tlb_tag [0:TLBN-1];   // VPN[26:0] (vpn2,vpn1,vpn0)
   reg [43:0]       tlb_ppn [0:TLBN-1];   // page PPN (leaf)
   reg [1:0]        tlb_lvl [0:TLBN-1];   // leaf level (0=4K,1=2M,2=1G)
   reg [7:0]        tlb_perm[0:TLBN-1];   // PTE perm bits V,R,W,X,U,G,A,D
   integer t;
   initial for (t=0;t<TLBN;t=t+1) tlb_v[t]=1'b0;

   wire [TLBI-1:0] tlb_idx = vpn0[TLBI-1:0];
   wire [26:0]     vpn_all = {vpn2, vpn1, vpn0};
   wire            tlb_hit = tlb_v[tlb_idx] && (tlb_tag[tlb_idx] == vpn_all);

   // assemble the translated physical address from a leaf entry by level
   function [AW-1:0] leaf_pa;
      input [43:0] ppn; input [1:0] lvl; input [63:0] va;
      begin
         case (lvl)
           2'd2: leaf_pa = {ppn[43:18], va[29:0]};   // 1 GiB
           2'd1: leaf_pa = {ppn[43:9],  va[20:0]};   // 2 MiB
           default: leaf_pa = {ppn, va[11:0]};       // 4 KiB
         endcase
      end
   endfunction

   // permission check for a leaf PTE `p` (bits V,R,W,X,U,G,A,D)
   function perm_fault;
      input [63:0] p; input [1:0] acc; input [1:0] prv; input s_sum, s_mxr;
      reg r,w,x,u,a,d;
      begin
         r=p[1]; w=p[2]; x=p[3]; u=p[4]; a=p[6]; d=p[7];
         perm_fault =
            (acc==2'd0 && !x)                                   ||  // fetch needs X
            ((acc==2'd1||acc==2'd3) && !r && !(s_mxr && x))     ||  // load/amo needs R (or X w/ MXR)
            ((acc==2'd2||acc==2'd3) && !w)                      ||  // store/amo needs W
            (prv==2'd0 && !u)                                   ||  // U-mode needs U
            (prv==2'd1 && u && (acc==2'd0 || !s_sum))           ||  // S-mode + U page
            (!a || (acc>=2'd2 && !d));                              // RVA22 software A/D
      end
   endfunction

   // -------------------- walk FSM --------------------
   localparam IDLE=2'd0, REQ=2'd1, RCV=2'd2;
   reg [1:0]  st;
   reg [1:0]  lvl;
   reg [43:0] walk_ppn;       // current table PPN
   reg [63:0] va_q;
   reg [1:0]  acc_q, prv_q;
   reg        sum_q, mxr_q;
   // cause for the in-flight walk (access type was latched at start)
   wire [3:0] pf_cause_q = (acc_q == 2'd0) ? 4'd12 : (acc_q == 2'd1) ? 4'd13 : 4'd15;
   // registered result of a just-completed walk (presented for one cycle)
   reg        w_done;
   reg [AW-1:0] w_paddr;
   reg        w_fault;
   reg [3:0]  w_cause;
   initial begin st=IDLE; ptw_read=0; w_done=0; end

   wire hit_perm_fault = perm_fault({56'd0, tlb_perm[tlb_idx]}, req_access, priv, sum, mxr);

   // the in-flight (or just-finished) walk is for va_q; only honor its result when the
   // current request still matches (a redirect can change req_vaddr mid-walk -> the stale
   // walk must not be reported against the new request).
   wire req_match = (va_q == req_vaddr);

   // ---- combinational translation result ----
   // resolves this cycle on: Bare, non-canonical, a TLB hit, or a just-finished walk.
   assign t_ready = req_valid & (!xlate | noncanon | tlb_hit | (w_done & req_match));
   wire wdm = w_done & req_match;
   assign t_paddr = wdm      ? w_paddr :
                    !xlate    ? req_vaddr[AW-1:0] :
                                leaf_pa(tlb_ppn[tlb_idx], tlb_lvl[tlb_idx], req_vaddr);
   assign t_fault = wdm ? w_fault : (noncanon | (tlb_hit & hit_perm_fault));
   assign t_cause = wdm ? w_cause : pf_cause;

   // start a walk when the request can't resolve this cycle
   wire start_walk = req_valid & xlate & !noncanon & !tlb_hit & !wdm & (st==IDLE);

   // PTE address = (table_ppn << 12) | (vpn[lvl] << 3)
   wire [8:0] vpn_lvl = (lvl==2'd2) ? va_q[38:30] : (lvl==2'd1) ? va_q[29:21] : va_q[20:12];
   wire [AW-1:0] pte_addr = {walk_ppn, 12'd0} | {vpn_lvl, 3'd0};

   integer fl;
   always @(posedge clk) begin
      if (reset) begin
         st<=IDLE; ptw_read<=1'b0; w_done<=1'b0;
         for (fl=0; fl<TLBN; fl=fl+1) tlb_v[fl]<=1'b0;
      end else begin
         w_done<=1'b0; ptw_read<=1'b0;
         if (flush) for (fl=0; fl<TLBN; fl=fl+1) tlb_v[fl]<=1'b0;
         case (st)
           IDLE: if (start_walk) begin
              va_q<=req_vaddr; acc_q<=req_access; prv_q<=priv; sum_q<=sum; mxr_q<=mxr;
              walk_ppn<=satp[43:0]; lvl<=2'd2; st<=REQ;
           end
           REQ: if (!req_match) st<=IDLE;           // request changed -> abort stale walk
                else begin ptw_addr<=pte_addr; ptw_read<=1'b1; st<=RCV; end
           RCV: if (!req_match) st<=IDLE;           // request changed -> abort stale walk
                else if (ptw_rvalid) begin
              // ptw_rdata = the PTE
              if (!ptw_rdata[0] || (!ptw_rdata[1] && ptw_rdata[2])) begin
                 w_fault<=1'b1; w_cause<=pf_cause_q; w_done<=1'b1; st<=IDLE;   // invalid
              end else if (ptw_rdata[1] || ptw_rdata[3]) begin                 // leaf (R|X)
                 if (((lvl==2'd2) && (ptw_rdata[27:10]!=0)) ||
                     ((lvl==2'd1) && (ptw_rdata[18:10]!=0))) begin
                    w_fault<=1'b1; w_cause<=pf_cause_q; w_done<=1'b1; st<=IDLE; // misaligned superpage
                 end else if (perm_fault(ptw_rdata, acc_q, prv_q, sum_q, mxr_q)) begin
                    w_fault<=1'b1; w_cause<=pf_cause_q; w_done<=1'b1; st<=IDLE;
                 end else begin
                    w_paddr <= leaf_pa(ptw_rdata[53:10], lvl, va_q);
                    w_fault<=1'b0; w_done<=1'b1; st<=IDLE;
                    // fill TLB
                    tlb_v[va_q[12+:TLBI]]   <= 1'b1;
                    tlb_tag[va_q[12+:TLBI]] <= va_q[38:12];
                    tlb_ppn[va_q[12+:TLBI]] <= ptw_rdata[53:10];
                    tlb_lvl[va_q[12+:TLBI]] <= lvl;
                    tlb_perm[va_q[12+:TLBI]]<= ptw_rdata[7:0];
                 end
              end else if (lvl==2'd0) begin
                 w_fault<=1'b1; w_cause<=pf_cause_q; w_done<=1'b1; st<=IDLE;    // no leaf at level 0
              end else begin
                 walk_ppn<=ptw_rdata[53:10]; lvl<=lvl-1'b1; st<=REQ;           // descend
              end
           end
         endcase
      end
   end
endmodule

`default_nettype wire
