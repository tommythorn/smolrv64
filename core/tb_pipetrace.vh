   // ---- +kanata=<file>: the pipe trace (Konata's format; tools/pipeview reads it) ------------
   // One row per fetched instruction and ONE CODE PER CYCLE: the stage it advanced to, or what holds
   // it, read from the hardware's own state that cycle (docs/SmolRV64-Spec.md §11.2 lists the codes;
   // tools/pipeview's legend explains them). A row is keyed by its fetch sequence number until
   // dispatch binds a ROB index. Two more kinds of row cover the cycles no instruction explains: a
   // FETCH row per run of cycles the bundle register could take a bundle and fetch has none, named by
   // the frontend's reason, and a STALL row per run of non-producing cycles, named by the core's
   // top-down cause (td_k). Honors +trace_from/+trace_to: the window's end and the run's close the
   // open fetch and stall rows. tools/kanata-disasm.py swaps the raw instruction words for
   // disassembly from the ELF.
   integer    kf;  reg kan_on;  reg [63:0] kan_last, kan_next, kan_rid;
   reg [8*256-1:0] kan_path;
   localparam integer PT_FW = `SMOLRV64_FW;          // the bundle register's slots
   localparam integer PT_QD = `SMOLRV64_QD;          // the decoupling queue's depth (QDEPTH)
   // rows by ROB index (after dispatch: 256 entries, the core's ROB_DEPTH at most) and by sequence
   // number (before: PT_SB bits, the core's SEQW at most) -- both checked at the start
   localparam integer PT_SB = 10;
   localparam integer PT_SN = 1 << PT_SB;
   reg [63:0] pt_id [0:255];  reg pt_v [0:255];  reg [7:0] pt_c [0:255];
   // a ROB tag's storage index, the rows' key: the tag without its wrap bit (smolrv64_rob)
   localparam integer PT_CB = (IW > 1) ? $clog2(IW) : 1;
   function [7:0] ptx(input [15:0] t);
      ptx = 8'(((t >> (PT_CB + 1)) << PT_CB) | (t & ((16'd1 << PT_CB) - 16'd1)));
   endfunction
   reg [63:0] ps_id [0:PT_SN-1];  reg ps_v [0:PT_SN-1];  reg [7:0] ps_c [0:PT_SN-1];
   reg [63:0] ps_pc [0:PT_SN-1];  reg [31:0] ps_raw [0:PT_SN-1];
   // mirrors of the bundle register and the decoupling queue: the sequence numbers they hold
   reg [PT_SB-1:0] pbm_seq [0:PT_FW-1];  reg [PT_FW-1:0] pbm_v;
   reg [63:0]      pbm_pc [0:PT_FW-1];   reg [31:0] pbm_raw [0:PT_FW-1];
   reg [PT_SB-1:0] qm_seq [0:PT_QD-1];   integer qm_n;
   // the fetch and stall rows
   reg [63:0] kan_sid, kan_gid;  reg kan_sopen, kan_gopen;  reg [4:0] kan_sk;  reg [8*12-1:0] kan_gk;
   integer    kti;
   initial begin
      kan_on = $value$plusargs("kanata=%s", kan_path);
      kan_last = 64'd0; kan_next = 64'd0; kan_rid = 64'd0;
      kan_sopen = 1'b0; kan_gopen = 1'b0; kan_sk = 5'd31; kan_gk = "";
      pbm_v = {PT_FW{1'b0}};  qm_n = 0;
      for (kti = 0; kti < 256; kti = kti + 1) pt_v[kti] = 1'b0;
      for (kti = 0; kti < PT_SN; kti = kti + 1) ps_v[kti] = 1'b0;
      if (dut.core.ROB_DEPTH > 256 || dut.core.SEQW > PT_SB)
         $fatal(1, "pipe trace: the core's ROB_DEPTH %0d / SEQW %0d exceed the trace's 256 rows / %0d bits",
                dut.core.ROB_DEPTH, dut.core.SEQW, PT_SB);
      if (kan_on) begin
         kf = $fopen(kan_path, "w");
         if (kf == 0) begin $display("FATAL: cannot open +kanata=%0s", kan_path); $finish; end
         $fdisplay(kf, "Kanata\t0004");
         $fdisplay(kf, "C=\t0");
      end
   end
   task kan_tick;                 // advance the view's clock to this cycle
      begin
         if (c != kan_last) begin $fdisplay(kf, "C\t%0d", c - kan_last); kan_last = c; end
      end
   endtask
   task kan_open;                 // a new row: its label and its first code
      input [63:0] id; input [8*40-1:0] label; input [7:0] code;
      begin
         kan_tick;
         $fdisplay(kf, "I\t%0d\t%0d\t0", id, id);
         $fdisplay(kf, "L\t%0d\t0\t%0s", id, label);
         $fdisplay(kf, "S\t%0d\t0\t%c", id, code);
      end
   endtask
   task kan_code;                 // the row's code changes
      input [63:0] id; input [7:0] old; input [7:0] code;
      begin
         if (code != old) begin
            kan_tick;
            $fdisplay(kf, "E\t%0d\t0\t%c", id, old);
            $fdisplay(kf, "S\t%0d\t0\t%c", id, code);
         end
      end
   endtask
   task kan_close;                // retired (0) or flushed (1) at the end of this cycle
      input [63:0] id; input [7:0] old; input flushed;
      begin
         kan_tick;
         $fdisplay(kf, "E\t%0d\t0\t%c", id, old);
         $fdisplay(kf, "R\t%0d\t%0d\t%0d", id, kan_rid, flushed);
         kan_rid = kan_rid + 64'd1;
      end
   endtask
   task ps_new;                   // a fetched instruction enters the bundle register
      input [PT_SB-1:0] sq; input [63:0] pc; input [31:0] raw;
      reg [8*40-1:0] lb;
      begin
         if (ps_v[sq]) begin kan_close(ps_id[sq], ps_c[sq], 1'b1); ps_v[sq] = 1'b0; end
         ps_id[sq] = kan_next;  kan_next = kan_next + 64'd1;  ps_v[sq] = 1'b1;  ps_c[sq] = "f";
         ps_pc[sq] = pc;  ps_raw[sq] = raw;
         $sformat(lb, "%h: %h", pc, raw);
         kan_open(ps_id[sq], lb, "f");
      end
   endtask

   // ---- the frontend's codes: the bundle register, the queue, its head group, the IR ----
   // Why queue-head slot k does not pop this cycle (it is in the head group's window). The head
   // (slot A) waiting on a credit is the back-end's cause, so its code takes td_k's conditions in
   // td_k's order -- the ROB, a dependency, the scheduler credits (its lane's, the head-op register,
   // the FP queue, the MD stage), rename, the SQ, the LQ, serialisation -- and names what the stall
   // row names. Slot k > 0 behind a dispatching slot is cut by a group rule or a room credit.
   function [7:0] head_why;
      input integer k;
      reg [15:0] g;  reg [CRW-1:0] cr;
      begin
         g = dut.core.fe.g[k];  cr = dut.core.crd;
         if (dut.core.fe.take[k])                          head_why = "q";   // pops: in the IR next cycle
         else if (k > 0 && !dut.core.fe.take[k-1])         head_why = "q";   // an older one holds
         else if (!dut.core.fe.rule[k])                    head_why = "g";   // cut by a group rule
         else if (k == 0) begin
            if (dut.core.redirect_q | dut.core.fr_v | dut.core.dec_red_q) head_why = "r";   // dispatch frozen
            else if (dut.core.st_rob)                      head_why = "O";   // no ROB row
            else if (dut.core.dep_ld | dut.core.dep_fp)    head_why = "W";   // nothing issues: a load or FP wait
            else if (!dut.core.hd_sched)                   head_why = (g[dut.core.GC_L] & !dut.core.cr_l) ? "o" : "I";
            else if (dut.core.rn_stall)                    head_why = "n";   // a rename shard is low
            else if (dut.core.hd_sq)                       head_why = "S";
            else if (dut.core.hd_lq)                       head_why = "L";
            else                                           head_why = "z";   // serialisation, a CSR or CBO gate
         end
         else if (!cr[CR_ROB1 + k])                        head_why = "O";
         else if ((g[dut.core.GC_I] & !dut.core.fe.ia[k]) | (g[dut.core.GC_FC] & !cr[CR_F + k]) | (g[dut.core.GC_M] & !cr[CR_DIV]))
                                                           head_why = "I";   // its lane's scheduler, the FP queue, the MD stage
         else if (g[dut.core.GC_ST] & !cr[CR_ST + k])      head_why = "S";
         else if (g[dut.core.GC_LD] & !cr[CR_LD + k])      head_why = "L";
         else                                              head_why = "?";
      end
   endfunction

   // Why fetch hands the bundle register nothing in a cycle it could take a bundle (FK_*), the
   // fetch rows' and FETCH-SIM's one classifier: itlb is the iMMU without a translation of the PC
   // or with a fault for it, in-flight the ring empty with its room reserved by the pairs in flight.
   localparam integer FK_N = 12;
   function [3:0] fetch_k;
      input dummy;
      begin
         if (dut.core.fe.redirect)                                 fetch_k = 4'd0;
         else if (!dut.core.fe.u_fetch.imem_ok) begin
            if (dut.core.fe.u_ring.freeze)                         fetch_k = 4'd1;
            else if (!dut.core.fe.u_ring.xlate_ok)                 fetch_k = 4'd2;
            else if (!dut.core.fe.u_ring.inpg)                     fetch_k = 4'd3;
            else if (!dut.core.fe.pq_room)                         fetch_k = 4'd4;
            else if (!dut.core.fe.u_ring.ic_req)                   fetch_k = 4'd5;
            else if (!dut.core.fe.u_ring.ic_ack)                   fetch_k = 4'd6;
            else                                                   fetch_k = 4'd7;
         end
         else if (dut.core.fe.u_fetch.strad)                       fetch_k = 4'd8;
         else if (dut.core.fe.u_fetch.reject)                      fetch_k = 4'd9;
         else if (dut.core.fe.u_fetch.bytes_late)                  fetch_k = 4'd10;
         else                                                      fetch_k = 4'd11;
      end
   endfunction
   function [8*12-1:0] fetch_name;
      input [3:0] k;
      case (k)
        4'd0: fetch_name = "redirect";    4'd1: fetch_name = "freeze";      4'd2: fetch_name = "itlb";
        4'd3: fetch_name = "page";        4'd4: fetch_name = "pq-full";     4'd5: fetch_name = "in-flight";
        4'd6: fetch_name = "icache-busy"; 4'd7: fetch_name = "icache-wait"; 4'd8: fetch_name = "straddle";
        4'd9: fetch_name = "reject";      4'd10: fetch_name = "bytes-late"; default: fetch_name = "align";
      endcase
   endfunction
   // FETCH-SIM: every cycle the bundle register takes a bundle, is held (the queue full), or could
   // take one and fetch has none, by the reason; printed with TOPDOWN-SIM
   reg [63:0] fs_take, fs_held, fs_cnt [0:FK_N-1];
   initial begin fs_take = 0; fs_held = 0; for (kti = 0; kti < FK_N; kti = kti + 1) fs_cnt[kti] = 0; end
   always @(posedge clk) if (!reset && meas) begin
      if (!dut.core.fe.pb_ready)      fs_held <= fs_held + 1;
      else if (dut.core.fe.dq_valid)  fs_take <= fs_take + 1;
      else                            fs_cnt[fetch_k(1'b0)] <= fs_cnt[fetch_k(1'b0)] + 1;
   end

   // ---- each lane's state, flattened for the per-cycle loop (a generate index must be constant) ----
   localparam integer PT_NI = NI;                    // a lane scheduler's entries (smolrv64_geom.vh)
   wire [IW-1:0]           pl_stg_v, pl_iss_v, pl_x, pl_m1, pl_m2;
   wire [7:0]              pl_stg_rob [0:IW-1], pl_iss_ent [0:IW-1], pl_rob [0:IW-1];
   wire [7:0]              pl_mrob1 [0:IW-1], pl_mrob2 [0:IW-1];
   wire [IW*PT_NI-1:0]     pl_iq_v, pl_iq_rdy;
   wire [7:0]              pl_iq_rob [0:IW*PT_NI-1];
   genvar gpl, gpe;
   generate for (gpl = 0; gpl < IW; gpl = gpl + 1) begin: ptl
      assign pl_stg_v[gpl]   = dut.core.ln[gpl].stg_v;
      assign pl_stg_rob[gpl] = ptx(16'(dut.core.ln[gpl].stg_rob));
      assign pl_iss_v[gpl]   = dut.core.ln[gpl].iss_v;
      assign pl_iss_ent[gpl] = 8'(dut.core.ln[gpl].iss_ent);
      assign pl_x[gpl]       = dut.core.ln[gpl].v & ~dut.core.redirect;   // executes this cycle
      assign pl_rob[gpl]     = ptx(16'(dut.core.ln[gpl].rob));
      assign pl_m1[gpl]      = dut.core.ln[gpl].m1;   assign pl_mrob1[gpl] = ptx(16'(dut.core.ln[gpl].mrob1));
      assign pl_m2[gpl]      = dut.core.ln[gpl].m2;   assign pl_mrob2[gpl] = ptx(16'(dut.core.ln[gpl].mrob2));
      for (gpe = 0; gpe < PT_NI; gpe = gpe + 1) begin: e
         assign pl_iq_v[gpl*PT_NI + gpe]   = dut.core.ln[gpl].u_iq.v[gpe];
         assign pl_iq_rdy[gpl*PT_NI + gpe] = &dut.core.ln[gpl].u_iq.e_r[gpe];
         assign pl_iq_rob[gpl*PT_NI + gpe] = ptx(16'(dut.core.ln[gpl].u_iq.e_rob[gpe]));
      end
   end endgenerate

   // ---- the backend's codes, by ROB index: the latest source that names an entry wins ----
   reg [7:0] nc [0:255];
   integer   pi, pj, pk;
   task put;  input [7:0] ix; input [7:0] code; begin nc[ix] = code; end endtask

   always @(posedge clk) if (!reset && kan_on) begin : pipetrace
      reg [7:0] ix;  reg [8*12-1:0] gk;  reg [8*40-1:0] lb;  reg [4:0] sk;
      // 1. what holds or moves every dispatched instruction this cycle
      for (pi = 0; pi < 256; pi = pi + 1)
         if (pt_v[pi]) nc[pi] = dut.core.u_rob.done[pi] ? "=" : "?";
      // the lanes: the dispatch stage (a cycle on the way to the scheduler), and the scheduler
      // (waiting for an operand, ready, picked)
      for (pj = 0; pj < IW; pj = pj + 1) begin
         if (pl_stg_v[pj]) put(pl_stg_rob[pj], "t");
         for (pk = 0; pk < PT_NI; pk = pk + 1)
            if (pl_iq_v[pj*PT_NI + pk])
               put(pl_iq_rob[pj*PT_NI + pk], (pl_iss_v[pj] && pl_iss_ent[pj] == 8'(pk)) ? "p"
                                             : pl_iq_rdy[pj*PT_NI + pk] ? "s" : "w");
      end
      // the FP port: its dispatch stage, its scheduler, j_*, the lanes' handoffs, stage F and the
      // FPU's ops in flight
      if (dut.core.stg_v_f) put(ptx(16'(dut.core.stg_rob_f)), "t");
      for (pj = 0; pj < dut.core.NF; pj = pj + 1)
         if (dut.core.u_iq_f.v[pj])
            put(ptx(16'(dut.core.u_iq_f.e_rob[pj])),
                (dut.core.u_iq_f.iss_v && 32'(dut.core.u_iq_f.iss_ent) == pj) ? "p"
                : (&dut.core.u_iq_f.e_r[pj]) ? "s" : "w");
      if (dut.core.j_v)     put(ptx(16'(dut.core.j_rob)), "j");
      for (pj = 0; pj < IW; pj = pj + 1) if (dut.core.hx_vv[pj]) put(ptx(16'(dut.core.hx_rob_l[pj])), "h");
      for (pj = 0; pj < dut.core.NFQ; pj = pj + 1) if (dut.core.fq_v[pj]) put(ptx(16'(dut.core.fq_rob[pj])), "F");
      if (dut.core.f_valid) put(ptx(16'(dut.core.f_rob)), "F");
      // memory: an address waiting for M, M's fill, a load waiting to access or accessing, a store
      // waiting for its data, a result waiting in its lane's landing buffer
      for (pj = 0; pj < dut.core.LQ_N; pj = pj + 1) if (dut.core.u_lq.v[pj]) begin
         ix = ptx(16'(dut.core.u_lq.rob[pj]));
         if (dut.core.u_lq.lnd[pj]) ;                   // landed and watched: complete
         else if (dut.core.la_v[pj])                    put(ix, "m");
         else if (dut.core.u_lq.av[pj] & dut.core.u_lq.sent[pj])  put(ix, "l");
         else if (dut.core.u_lq.av[pj] & !dut.core.u_rob.done[ix]) put(ix, "k");
      end
      for (pj = 0; pj < dut.core.SQ_N; pj = pj + 1) if (dut.core.u_sq.v[pj]) begin
         ix = ptx(16'(dut.core.u_sq.rob[pj]));
         if (dut.core.sa_v[pj])                         put(ix, "m");
         else if (dut.core.u_sq.av[pj] & ~dut.core.u_sq.dv[pj]) put(ix, "y");
         else if (dut.core.u_sq.av[pj])                 put(ix, "=");
      end
      if (dut.core.m_valid) put(ptx(16'(dut.core.m_rob_idx)), "M");
      for (pj = 0; pj < IW; pj = pj + 1)
         for (pk = 0; pk < dut.core.LBN; pk = pk + 1)
            if (pk < dut.core.lb_n[pj])
               put(ptx(16'(dut.core.lb_rob[pj*dut.core.LBN + ((dut.core.lb_h[pj] + pk) % dut.core.LBN)])), "e");
      if (dut.core.sy_v)  put(ptx(16'(dut.core.sy_rob)), "Y");
      if (dut.core.md_v)  put(ptx(16'(dut.core.md_rob)), "v");
      if (dut.core.mq_v)  put(ptx(16'(dut.core.mq_rob)), "v");
      // a lane's multiply in flight, and what it executes this cycle
      for (pj = 0; pj < IW; pj = pj + 1) begin
         if (pl_m1[pj]) put(pl_mrob1[pj], "u");
         if (pl_m2[pj]) put(pl_mrob2[pj], "u");
         if (pl_x[pj])  put(pl_rob[pj], 8'("a" + pj));
      end
      // a mispredicted CTI waiting for its restart (it completes at the squash or the release), and
      // the entries the kill marked dead, waiting for the release that drops them
      for (pj = 0; pj < IW; pj = pj + 1)
         if (dut.core.lr_v[pj] & dut.core.lr_mis[pj]) put(ptx(16'(dut.core.lr_rob[pj])), "!");
      if (dut.core.fr_v) put(ptx(16'(dut.core.fr_rob)), "!");
      if (dut.core.rob_kd_v)
         for (pi = 0; pi < dut.core.ROB_DEPTH; pi = pi + 1) if (dut.core.rob_kd[pi]) put(8'(pi), "X");
      if (trace_on) for (pi = 0; pi < 256; pi = pi + 1)
         if (pt_v[pi]) begin kan_code(pt_id[pi], pt_c[pi], nc[pi]); pt_c[pi] = nc[pi]; end

      // 2. the frontend's rows: the bundle register (a row opens the first cycle it is there), the
      // queue, the IR
      for (pj = 0; pj < PT_FW; pj = pj + 1)
         if (pbm_v[pj] && !ps_v[pbm_seq[pj]] && trace_on) ps_new(pbm_seq[pj], pbm_pc[pj], pbm_raw[pj]);
      for (pj = 0; pj < PT_FW; pj = pj + 1)
         if (pbm_v[pj] && ps_v[pbm_seq[pj]]) begin
            kan_code(ps_id[pbm_seq[pj]], ps_c[pbm_seq[pj]], (dut.core.fe.pb_v & ~dut.core.fe.q_room) ? "Q" : "f");
            ps_c[pbm_seq[pj]] = (dut.core.fe.pb_v & ~dut.core.fe.q_room) ? "Q" : "f";
         end
      for (pj = 0; pj < qm_n; pj = pj + 1)
         if (ps_v[qm_seq[pj]]) begin
            kan_code(ps_id[qm_seq[pj]], ps_c[qm_seq[pj]], (pj < IW) ? head_why(pj) : "q");
            ps_c[qm_seq[pj]] = (pj < IW) ? head_why(pj) : "q";
         end
      for (pj = 0; pj < IW; pj = pj + 1)
         if (dut.core.s_v[pj] && ps_v[dut.core.s_seq[pj]]) begin
            kan_code(ps_id[dut.core.s_seq[pj]], ps_c[dut.core.s_seq[pj]], dut.core.d_take ? "i" : "r");
            ps_c[dut.core.s_seq[pj]] = dut.core.d_take ? "i" : "r";
         end

      // 3. the ends of this cycle: retire, the squashes, dispatch binding a ROB index, the IR's
      // dropped group, a frontend redirect
      for (pj = 0; pj < IW; pj = pj + 1)
         if (dut.core.rc_v[pj] && pt_v[dut.core.rc_sx[pj]]) begin
            kan_close(pt_id[dut.core.rc_sx[pj]], pt_c[dut.core.rc_sx[pj]], 1'b0);  pt_v[dut.core.rc_sx[pj]] = 1'b0;
         end
      if (dut.core.redirect)
         for (pi = 0; pi < 256; pi = pi + 1) if (pt_v[pi]) begin kan_close(pt_id[pi], pt_c[pi], 1'b1); pt_v[pi] = 1'b0; end
      if (dut.core.fr_rel)
         for (pi = 0; pi < dut.core.ROB_DEPTH; pi = pi + 1)
            if (dut.core.rob_kd[pi] && pt_v[pi]) begin kan_close(pt_id[pi], pt_c[pi], 1'b1); pt_v[pi] = 1'b0; end
      for (pj = 0; pj < IW; pj = pj + 1)
         if (dut.core.s_v[pj] && ps_v[dut.core.s_seq[pj]]) begin
            if (dut.core.s_takev[pj]) begin
               ix = ptx(16'(dut.core.s_rob[pj]));
               if (pt_v[ix]) kan_close(pt_id[ix], pt_c[ix], 1'b1);
               pt_id[ix] = ps_id[dut.core.s_seq[pj]];  pt_c[ix] = ps_c[dut.core.s_seq[pj]];  pt_v[ix] = 1'b1;
               $fdisplay(kf, "L\t%0d\t1\trob %0d seq %0d", pt_id[ix], ix, dut.core.s_seq[pj]);   // the detail: grep a row by either
            end else
               kan_close(ps_id[dut.core.s_seq[pj]], ps_c[dut.core.s_seq[pj]], 1'b1);
            ps_v[dut.core.s_seq[pj]] = 1'b0;
         end
      // the queue's mirror holds what the queue holds this cycle, before the edge below moves both
      if (qm_n != 32'(dut.core.fe.q_cnt) && trace_on)
         $display("pipetrace: the queue mirror holds %0d, the queue %0d (cycle %0d)", qm_n, dut.core.fe.q_cnt, c);
      // the mirrors follow the hardware's edges: a redirect empties the bundle register and the
      // queue; else the queue pops the head group, takes the bundle register's slots on a push, and
      // the bundle register loads a bundle from fetch
      if (dut.core.fe.redirect) begin
         for (pj = 0; pj < PT_FW; pj = pj + 1)
            if (pbm_v[pj] && ps_v[pbm_seq[pj]]) begin kan_close(ps_id[pbm_seq[pj]], ps_c[pbm_seq[pj]], 1'b1); ps_v[pbm_seq[pj]] = 1'b0; end
         for (pj = 0; pj < qm_n; pj = pj + 1)
            if (ps_v[qm_seq[pj]]) begin kan_close(ps_id[qm_seq[pj]], ps_c[qm_seq[pj]], 1'b1); ps_v[qm_seq[pj]] = 1'b0; end
         pbm_v = {PT_FW{1'b0}};  qm_n = 0;
      end else begin
         pk = 32'(dut.core.fe.q_pop);
         for (pj = 0; pj + pk < qm_n; pj = pj + 1) qm_seq[pj] = qm_seq[pj + pk];
         qm_n = qm_n - pk;
         if (dut.core.fe.q_push) begin
            for (pj = 0; pj < PT_FW; pj = pj + 1)
               if (pbm_v[pj]) begin qm_seq[qm_n] = pbm_seq[pj]; qm_n = qm_n + 1; end
            pbm_v = {PT_FW{1'b0}};
         end
         if (dut.core.fe.pb_load & dut.core.fe.dq_valid) begin
            for (pj = 0; pj < PT_FW; pj = pj + 1) begin
               pbm_v[pj] = dut.core.fe.dq_sv[pj];
               pbm_seq[pj] = PT_SB'((dut.core.fe.dq_seq >> (pj * dut.core.SEQW)) & ((64'd1 << dut.core.SEQW) - 64'd1));
               pbm_pc[pj]  = 64'($signed(dut.core.fe.dq_pc[pj*dut.core.PCW +: dut.core.PCW]));   // a PC's sign extension
               pbm_raw[pj] = dut.core.fe.dq_inst[pj*32 +: 32];
            end
         end
      end

      // 4. the fetch row: one per run of cycles the bundle register could take a bundle and fetch
      // offers none, named by the reason
      gk = (trace_on && dut.core.fe.pb_ready && !dut.core.fe.dq_valid) ? fetch_name(fetch_k(1'b0)) : "";
      if (gk != kan_gk) begin
         if (kan_gopen) begin kan_close(kan_gid, "~", 1'b1); kan_gopen = 1'b0; end
         if (gk != "") begin
            kan_gid = kan_next;  kan_next = kan_next + 64'd1;  kan_gopen = 1'b1;
            $sformat(lb, "~~ fetch: %0s", gk);
            kan_open(kan_gid, lb, "~");
         end
      end
      kan_gk = gk;
      // 5. the stall row: one per run of cycles with the same non-producing cause (td_k 3..20);
      // the window's end closes it (31: outside the window)
      sk = trace_on ? td_k : 5'd31;
      if (sk != kan_sk) begin
         if (kan_sopen) begin kan_close(kan_sid, "#", 1'b1); kan_sopen = 1'b0; end
         if (sk >= 5'd3 && sk <= 5'd20) begin
            kan_sid = kan_next;  kan_next = kan_next + 64'd1;  kan_sopen = 1'b1;
            $sformat(lb, "** %0s", td_name(sk));
            kan_open(kan_sid, lb, "#");
         end
      end
      kan_sk = sk;
   end
   task kan_finish;               // the run ends: close the open fetch and stall rows
      begin
         if (kan_gopen) begin kan_close(kan_gid, "~", 1'b1); kan_gopen = 1'b0; end
         if (kan_sopen) begin kan_close(kan_sid, "#", 1'b1); kan_sopen = 1'b0; end
      end
   endtask
