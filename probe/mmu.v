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
    parameter TLBI = 4,            // clog2(TLBN)
    // Valid-DRAM window for the physical-address check below. Default is fully
    // permissive (base 0, unbounded top) so unit TBs / FPGA see no new faults; the
    // cosim build narrows it to the modeled DDR so an out-of-range PA access-faults
    // exactly as the simmerv golden model does.
    parameter [63:0] DRAM_BASE = 64'd0,
    parameter [63:0] DRAM_TOP  = 64'hFFFF_FFFF_FFFF_FFFF)
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
    output wire [3:0]  t_cause,       // 12=instr, 13=load, 15=store page fault
    output wire        t_uncached);   // Svpbmt: leaf PBMT(pte[62:61])!=0 -> NC/IO (don't cache)

   wire        xlate = (satp[63:60] == 4'd8);   // 8 = Sv39, else Bare (identity)

   wire [8:0]  vpn2 = req_vaddr[38:30];
   wire [8:0]  vpn1 = req_vaddr[29:21];
   wire [8:0]  vpn0 = req_vaddr[20:12];
   wire        noncanon = (req_vaddr[63:39] != {25{req_vaddr[38]}});

   wire [3:0]  pf_cause = (req_access == 2'd0) ? 4'd12 :
                          (req_access == 2'd1) ? 4'd13 : 4'd15;
   // Bare mode has no page tables, so an out-of-range (non-canonical/unaddressable)
   // access raises an ACCESS fault, not a page fault.  cause 1=fetch, 5=load, 7=store/amo.
   // (matches SmolRV64's phys-region default case; PMP itself is intentionally unsupported.)
   wire [3:0]  af_cause = (req_access == 2'd0) ? 4'd1 :
                          (req_access == 2'd1) ? 4'd5 : 4'd7;

   // -------- physical-address validity (mirrors the simmerv golden memory map) --------
   // A resolved PA (identity in Bare, leaf in Sv39) that is neither RAM nor a mapped
   // device is unaddressable -> ACCESS fault, exactly as simmerv's load/store_mmio returns
   // Err for any PA outside {RAM | CLINT | PLIC | UART}.  The DRAM window is parameterized
   // (cosim sizes it from the modeled DDR; default unbounded = no fault); the device ranges
   // are fixed SoC constants mirrored from soc_top, and the on-chip boot SRAM at LBASE is
   // included so the FPGA monitor's own fetches/loads don't fault (never touched in cosim).
   localparam [63:0] CLINT_LO=64'h0200_0000, CLINT_HI=64'h0201_0000;   // 64 KiB
   localparam [63:0] PLIC_LO =64'h0c00_0000, PLIC_HI =64'h1000_0000;   // 64 MiB
   localparam [63:0] UART_LO =64'h1000_0000, UART_HI =64'h1000_0008;   // 8 NS16550 byte regs
   localparam [63:0] LSRAM_LO=64'h7000_0000, LSRAM_HI=64'h7004_0000;   // 256 KiB on-chip SRAM
   localparam [63:0] VIRTIO_LO=64'h1000_2000, VIRTIO_HI=64'h1000_4000; // virtio-mmio 8 KiB (blk+net)
   function pa_valid;
      input [63:0] pa;
      pa_valid = (pa >= DRAM_BASE && pa < DRAM_TOP)
              || (pa >= CLINT_LO  && pa < CLINT_HI)
              || (pa >= PLIC_LO   && pa < PLIC_HI)
              || (pa >= UART_LO   && pa < UART_HI)
              || (pa >= LSRAM_LO  && pa < LSRAM_HI)
              || (pa >= VIRTIO_LO && pa < VIRTIO_HI);
   endfunction

   // -------------------- TLB (direct-mapped on VPN[3:0] of vpn0) --------------------
   reg              tlb_v   [0:TLBN-1];
   reg [26:0]       tlb_tag [0:TLBN-1];   // VPN[26:0] (vpn2,vpn1,vpn0)
   reg [43:0]       tlb_ppn [0:TLBN-1];   // page PPN (leaf)
   reg [1:0]        tlb_lvl [0:TLBN-1];   // leaf level (0=4K,1=2M,2=1G)
   reg [7:0]        tlb_perm[0:TLBN-1];   // PTE perm bits V,R,W,X,U,G,A,D
   reg              tlb_nc  [0:TLBN-1];   // Svpbmt: leaf is NC/IO (PBMT pte[62:61] != 0)
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
   reg [63:0] satp_q;
   // cause for the in-flight walk (access type was latched at start)
   wire [3:0] pf_cause_q = (acc_q == 2'd0) ? 4'd12 : (acc_q == 2'd1) ? 4'd13 : 4'd15;
   // registered result of a just-completed walk (presented for one cycle)
   reg        w_done;
   reg [AW-1:0] w_paddr;
   reg        w_fault;
   reg [3:0]  w_cause;
   reg        w_nc;                  // Svpbmt: just-walked leaf is NC/IO
   initial begin st=IDLE; ptw_read=0; w_done=0; end

   wire hit_perm_fault = perm_fault({56'd0, tlb_perm[tlb_idx]}, req_access, priv, sum, mxr);

   // The in-flight (or just-finished) walk latched its whole CONTEXT (va, access,
   // priv, SUM/MXR, satp) at start; only honor its result while the current request
   // still matches ALL of it. Address alone is NOT enough: a serialized CSR write
   // can change the translation context mid-walk while the redirect's REFETCHED op
   // -- same instruction, same vaddr -- already holds the request. Observed: the
   // kernel's uaccess `csrw sstatus(SUM=1); sd <user-va>` -- the squashed sd's walk
   // (started under SUM=0) completed after the refetch and its stale perm-fault
   // verdict (perm_fault uses the _q context, and a faulting walk fills no TLB)
   // was delivered against the now-legal refetched store: spurious cause-15,
   // Ubuntu cosim divergence @111.6M. On a poisoned result the walk reruns under
   // the live context.
   //
   // The context compare is REGISTERED (ctx_poison), not part of the combinational
   // req_match: t_ready feeds the fetch/LSU stall cones, and a 64-bit live satp
   // equality there cost WNS -0.23/TNS -159 on the FPGA. The flop delays staleness
   // detection by one cycle; the only exposure is a walk COMPLETING the very cycle
   // after the CSR write's own EX edge, and that cycle is already covered by the
   // squash-side guards (lsu df_set suppresses a same-cycle-rollback latch, and
   // dflt_ready & ~eb_redirect defers delivery past an older redirect) -- the
   // refetched op's re-request arrives many cycles after the pulse regardless.
   reg ctx_poison;
   initial ctx_poison = 1'b0;
   wire ctx_stale = (prv_q != priv) | (sum_q != sum) | (mxr_q != mxr) | (satp_q != satp);
   wire req_match = (va_q == req_vaddr) & (acc_q == req_access) & ~ctx_poison;

   // ---- combinational translation result ----
   // resolves this cycle on: Bare, non-canonical, a TLB hit, or a just-finished walk.
   assign t_ready = req_valid & (!xlate | noncanon | tlb_hit | (w_done & req_match));
   wire wdm = w_done & req_match;
   assign t_paddr = wdm      ? w_paddr :
                    !xlate    ? req_vaddr[AW-1:0] :
                                leaf_pa(tlb_ppn[tlb_idx], tlb_lvl[tlb_idx], req_vaddr);
   // base (translation) fault: page/perm fault (Sv39) or non-canonical (Bare).
   wire        base_fault = wdm ? w_fault : (noncanon | (tlb_hit & hit_perm_fault));
   wire [3:0]  base_cause = wdm ? w_cause : (xlate ? pf_cause : af_cause);
   // PA-validity fault: only when the translation actually RESOLVES this cycle (t_ready) and
   // didn't already fault -- an unbacked resolved PA is an access fault.  t_ready-gating is
   // essential: mid-walk t_paddr is a stale leaf and must not raise a (spurious) fault.
   wire        pa_ok = pa_valid({{(64-AW){1'b0}}, t_paddr});
   assign t_fault = base_fault | (t_ready & ~pa_ok);
   assign t_cause = base_fault ? base_cause : af_cause;
   // Svpbmt memory type: NC/IO leaf -> uncached. Bare/non-canonical = normal (cacheable);
   // MMIO device regions are routed around the D$ by soc_top's address decode, not here.
   assign t_uncached = wdm ? w_nc : (xlate & tlb_hit & tlb_nc[tlb_idx]);

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
              satp_q<=satp; ctx_poison<=1'b0;
              walk_ppn<=satp[43:0]; lvl<=2'd2; st<=REQ;
           end
           REQ: if (!req_match) st<=IDLE;           // request changed -> abort stale walk
                else begin ptw_addr<=pte_addr; ptw_read<=1'b1; st<=RCV; end
           RCV: if (ptw_rvalid) begin
              // DRAIN the in-flight PTW read before honoring an abort: leaving RCV while a read
              // is still outstanding leaks its response into the NEXT walk's read (a stale wrong
              // PTE that can misdecode as a leaf -> spurious page fault). So abort only after the
              // response arrives (single-outstanding PTW port). req changed -> abort, read drained.
              if (!req_match) st<=IDLE;
              // ptw_rdata = the PTE
              else if (!ptw_rdata[0] || (!ptw_rdata[1] && ptw_rdata[2])) begin
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
                    w_nc   <= ptw_rdata[62] | ptw_rdata[61];   // Svpbmt PBMT != 0
                    // fill TLB
                    tlb_v[va_q[12+:TLBI]]   <= 1'b1;
                    tlb_tag[va_q[12+:TLBI]] <= va_q[38:12];
                    tlb_ppn[va_q[12+:TLBI]] <= ptw_rdata[53:10];
                    tlb_lvl[va_q[12+:TLBI]] <= lvl;
                    tlb_perm[va_q[12+:TLBI]]<= ptw_rdata[7:0];
                    tlb_nc[va_q[12+:TLBI]]  <= ptw_rdata[62] | ptw_rdata[61];
                 end
              end else if (lvl==2'd0) begin
                 w_fault<=1'b1; w_cause<=pf_cause_q; w_done<=1'b1; st<=IDLE;    // no leaf at level 0
              end else begin
                 walk_ppn<=ptw_rdata[53:10]; lvl<=lvl-1'b1; st<=REQ;           // descend
              end
           end
         endcase
         // context-staleness tracking (registered; see req_match). Runs every cycle
         // except a walk start (whose fresh _q latch + poison clear wins), including
         // the w_done presentation cycle after st has returned to IDLE.
         if (!(st==IDLE && start_walk) && ctx_stale) ctx_poison <= 1'b1;
      end
   end
endmodule

`default_nettype wire
