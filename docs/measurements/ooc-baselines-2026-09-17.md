# Memory-backend baselines, 2026-09-17 (program C0 / B8)

Out-of-context synthesis at the instance parameters (`make ooc MODULE=... PERIOD=6.0`, rule I2),
before any backend structure is replaced. Fmax is the module alone; the flat build is route-bound
(65-83% of the path), so these bound a structure's depth, not its placed slack.

| module | generics | WNS at 6 ns | Fmax |
|---|---|---|---|
| ooo2_lq (lq) | `NENT=4 IDXB=2 PAW=56 PBITS=10 ROBB=4 SQIB=4` | +4.344 ns | 603.9 MHz |
| ooo2_sq (sq) | `NENT=8 IDXB=3 PAW=56 PBITS=10 ROBB=4 NWB=5 LQN=4 LQIB=2` | +1.829 ns | 239.8 MHz |
| ooo2_lsu (lsu) | `AW=64` | +1.770 ns | 236.4 MHz |
| rv_cache (dcache) | `PAW=64 PAW_SIG=34 SIZE_KB=64 RDW=64 WDW=64 WRITABLE=1 WRTHRU=0 PREFETCH=0 PERF_ID=1` | +1.068 ns | 202.8 MHz |
| rv_cache (icache) | `PAW=64 PAW_SIG=39 SIZE_KB=64 RDW=128 WDW=64 WRITABLE=0 PREFETCH=1 VIRT=1 PERF_ID=0` | +1.058 ns | 202.3 MHz |
| ooo2_sq (sq16) | `NENT=16 IDXB=4 PAW=56 PBITS=10 ROBB=5 NWB=5 LQN=16 LQIB=4` | +1.929 ns | 245.6 MHz |
| ooo2_lq (lq16) | `NENT=16 IDXB=4 PAW=56 PBITS=10 ROBB=5 SQIB=5` | +3.455 ns | 392.9 MHz |

The 16-entry queue shapes the program moves to (ROB 32 / LQ 16 / SQ 16) are the `sq16`/`lq16` rows.

## Census of the last passing IW=2 build (W1' + the D12 fix, WNS +0.039, `scratchpad/census-f1iw2.txt`)

Worst families (unique endpoints under +0.35 ns):

```
   0.039      8     21  1.545  4.057  probe_core/core/m_addr_reg/C                 -> probe_core/core/hpm_ev_q_reg/D
   0.068     18     16  1.160  4.554  probe_core/core/ps_out_a2_reg/C              -> probe_core/core/u_sq/wb_q_reg
   0.070     14     13  1.072  4.769  probe_core/core/ps_out_reg_replica_3/C       -> probe_core/core/m_result_reg/D
   0.076     22     19  1.246  4.547  probe_core/core/ps_out_reg_replica_1/C       -> probe_core/core/m_result_reg/D
   0.084      8     21  1.161  4.471  probe_core/core/ps_out_a_reg/C               -> probe_core/core/u_sq/wb_q_reg
   0.086      3     16  1.189  4.433  probe_core/core/u_rename/rnewer_reg          -> probe_core/core/stg_r_l_reg/D
   0.088     10     22  1.446  4.064  probe_core/dev_rvalid_q_reg/C                -> probe_core/core/ps_out_reg_replica_2/CE
   0.088     24     20  1.194  4.477  probe_core/core/ps_out_f_reg/C               -> probe_core/core/cf_link_reg/D
   0.094      1     13  1.097  4.807  probe_core/core/ps_out_reg_replica_2/C       -> probe_core/core/m_result_reg/D
   0.095     39     17  1.151  4.567  probe_core/core/ps_out_a2_reg/C              -> probe_core/core/alu2_q_val_reg/D
```

```
=== slack histogram (probe_clk, unique endpoints) ===
slack < +0.00 : 0 endpoints
slack < +0.10 : 19 endpoints
slack < +0.20 : 212 endpoints
slack < +0.30 : 755 endpoints
slack < +0.40 : 1327 endpoints
slack < +0.50 : 2071 endpoints
slack < +0.75 : 7854 endpoints
slack < +1.00 : 11368 endpoints
```

The worst family is `m_addr_reg -> hpm_ev_q_reg` (21 levels): the counter bus carries
`m_valid & m_is_mem & lsu_done`, whose `lsu_done` starts at the dTLB compare. The C0 memory
buckets were built from registered queue state for that reason; the IW=3 build of C0 is the check.

## Limit study with the backend knobs (B9, `tools/trace-limit.py`, the 13 GB5 board traces)

IW=3, W=16/32, a 12-cycle drain every 100 instructions (10 MPKI), 2% of loads missing for 36
cycles unless stated. Ceilings: perfect prediction otherwise, no dTLB, no structural hazard but
the load/store ports. Misses are spread uniformly by a hash, so they never overlap within a
32-entry window and MLP shows nothing here; the MSHR gain has to be read off the real cosim
sweep (B7). What the table does say: the one-access-per-cycle door alone lifts the integer
workloads 40-70% (aes-xts 0.77 -> 1.33, clang 0.77 -> 1.18, sqlite 0.90 -> 1.26,
text-compression 0.96 -> 1.37 at W16); a one-per-cycle store drain adds 5-20% on the
store-heavy ones (clang, pdf-rendering, text-rendering); W32 adds 10-20% everywhere; the FP
workloads (gaussian-blur, n-body, structure-from-motion, machine-learning, image-compression)
stay at or below 0.9 in every case: FP latency and width, not the memory backend.

```
== today: load-use 5, ld-port 4, st-port 2, misses 2% lat 36 mlp 1
aes-xts             IW3/W16   0.77  IW3/W32   0.80
camera              IW3/W16   0.78  IW3/W32   0.89
clang               IW3/W16   0.77  IW3/W32   0.85
gaussian-blur       IW3/W16   0.58  IW3/W32   0.71
html5               IW3/W16   0.84  IW3/W32   0.92
image-compression   IW3/W16   0.57  IW3/W32   0.59
machine-learning    IW3/W16   0.67  IW3/W32   0.82
n-body-physics      IW3/W16   0.56  IW3/W32   0.68
pdf-rendering       IW3/W16   1.17  IW3/W32   1.29
sqlite              IW3/W16   0.90  IW3/W32   0.98
structure-from-motion  IW3/W16   0.58  IW3/W32   0.71
text-compression    IW3/W16   0.96  IW3/W32   1.04
text-rendering      IW3/W16   0.75  IW3/W32   0.81
== pipelined LSU: load-use 4, ld-port 1, st-port 2, mlp 1
aes-xts             IW3/W16   1.33  IW3/W32   1.50
camera              IW3/W16   0.84  IW3/W32   1.18
clang               IW3/W16   1.18  IW3/W32   1.39
gaussian-blur       IW3/W16   0.61  IW3/W32   0.75
html5               IW3/W16   1.26  IW3/W32   1.45
image-compression   IW3/W16   0.82  IW3/W32   0.84
machine-learning    IW3/W16   0.71  IW3/W32   0.86
n-body-physics      IW3/W16   0.68  IW3/W32   0.81
pdf-rendering       IW3/W16   1.31  IW3/W32   1.47
sqlite              IW3/W16   1.26  IW3/W32   1.42
structure-from-motion  IW3/W16   0.61  IW3/W32   0.75
text-compression    IW3/W16   1.37  IW3/W32   1.59
text-rendering      IW3/W16   1.14  IW3/W32   1.33
== + MSHRs: mlp 4
aes-xts             IW3/W16   1.33  IW3/W32   1.50
camera              IW3/W16   0.84  IW3/W32   1.18
clang               IW3/W16   1.18  IW3/W32   1.39
gaussian-blur       IW3/W16   0.61  IW3/W32   0.75
html5               IW3/W16   1.26  IW3/W32   1.45
image-compression   IW3/W16   0.82  IW3/W32   0.84
machine-learning    IW3/W16   0.71  IW3/W32   0.86
n-body-physics      IW3/W16   0.68  IW3/W32   0.81
pdf-rendering       IW3/W16   1.31  IW3/W32   1.47
sqlite              IW3/W16   1.26  IW3/W32   1.42
structure-from-motion  IW3/W16   0.61  IW3/W32   0.75
text-compression    IW3/W16   1.37  IW3/W32   1.59
text-rendering      IW3/W16   1.14  IW3/W32   1.33
== + store drain 1/cycle
aes-xts             IW3/W16   1.35  IW3/W32   1.52
camera              IW3/W16   0.84  IW3/W32   1.18
clang               IW3/W16   1.34  IW3/W32   1.56
gaussian-blur       IW3/W16   0.61  IW3/W32   0.75
html5               IW3/W16   1.35  IW3/W32   1.54
image-compression   IW3/W16   0.85  IW3/W32   0.87
machine-learning    IW3/W16   0.71  IW3/W32   0.86
n-body-physics      IW3/W16   0.68  IW3/W32   0.82
pdf-rendering       IW3/W16   1.57  IW3/W32   1.79
sqlite              IW3/W16   1.33  IW3/W32   1.51
structure-from-motion  IW3/W16   0.61  IW3/W32   0.75
text-compression    IW3/W16   1.39  IW3/W32   1.61
text-rendering      IW3/W16   1.31  IW3/W32   1.51
== perfect memory (the old ceiling, load-use 5)
aes-xts             IW3/W16   1.62  IW3/W32   1.91
camera              IW3/W16   0.94  IW3/W32   1.35
clang               IW3/W16   1.54  IW3/W32   1.80
gaussian-blur       IW3/W16   0.64  IW3/W32   0.79
html5               IW3/W16   1.53  IW3/W32   1.77
image-compression   IW3/W16   0.90  IW3/W32   0.91
machine-learning    IW3/W16   0.76  IW3/W32   0.94
n-body-physics      IW3/W16   0.73  IW3/W32   0.87
pdf-rendering       IW3/W16   1.59  IW3/W32   1.83
sqlite              IW3/W16   1.50  IW3/W32   1.75
structure-from-motion  IW3/W16   0.64  IW3/W32   0.79
text-compression    IW3/W16   1.55  IW3/W32   1.84
text-rendering      IW3/W16   1.52  IW3/W32   1.77
== misses 5%: today
aes-xts             IW3/W16   0.67  IW3/W32   0.73
camera              IW3/W16   0.67  IW3/W32   0.79
clang               IW3/W16   0.71  IW3/W32   0.80
gaussian-blur       IW3/W16   0.52  IW3/W32   0.67
html5               IW3/W16   0.75  IW3/W32   0.84
image-compression   IW3/W16   0.50  IW3/W32   0.52
machine-learning    IW3/W16   0.59  IW3/W32   0.71
n-body-physics      IW3/W16   0.51  IW3/W32   0.63
pdf-rendering       IW3/W16   1.06  IW3/W32   1.18
sqlite              IW3/W16   0.80  IW3/W32   0.90
structure-from-motion  IW3/W16   0.52  IW3/W32   0.66
text-compression    IW3/W16   0.85  IW3/W32   0.93
text-rendering      IW3/W16   0.67  IW3/W32   0.73
== misses 5%: pipelined + mlp 4
aes-xts             IW3/W16   1.01  IW3/W32   1.20
camera              IW3/W16   0.72  IW3/W32   1.02
clang               IW3/W16   0.97  IW3/W32   1.21
gaussian-blur       IW3/W16   0.54  IW3/W32   0.72
html5               IW3/W16   1.04  IW3/W32   1.24
image-compression   IW3/W16   0.67  IW3/W32   0.71
machine-learning    IW3/W16   0.62  IW3/W32   0.76
n-body-physics      IW3/W16   0.60  IW3/W32   0.74
pdf-rendering       IW3/W16   1.17  IW3/W32   1.35
sqlite              IW3/W16   1.04  IW3/W32   1.22
structure-from-motion  IW3/W16   0.54  IW3/W32   0.71
text-compression    IW3/W16   1.13  IW3/W32   1.34
text-rendering      IW3/W16   0.91  IW3/W32   1.12
SWEEP_DONE
```

## Census of the C0 IW=3 build (WNS +0.025, `scratchpad/census-c0iw3b.txt`)

```
   0.032      6     16  1.736  3.893  probe_core/core/m_addr_reg/C                 -> probe_core/core/hpm_ev_q_reg/D
   0.060      5      9  0.856  4.518  probe_core/core/m_imm_reg_replica_2/C        -> probe_core/core/u_prf/mem_ld_reg_r7_0_63_56_62
   0.062      4      9  0.887  4.489  probe_core/core/m_imm_reg_replica_2/C        -> probe_core/core/u_prf/mem_ld_reg_r4_64_127_56_62
   0.065      1     16  1.206  4.697  probe_core/core/fe/d_rs1_reg                 -> probe_core/core/stg_r_l_reg/D
   0.070      1     16  1.157  4.746  probe_core/core/fe/d_rs1_reg                 -> probe_core/core/stg_r_f_reg/D
   0.073      2     10  0.764  4.728  probe_core/core/m_imm_reg_replica_2/C        -> probe_core/core/u_prf/mem_ld_reg_r5_64_127_63_63
   0.093     40     18  1.737  3.554  probe_core/core/fe/u_fetch                   -> probe_core/core/fe/u_bp
   0.098      4     17  1.512  3.913  probe_core/core/m_addr_reg/C                 -> probe_core/core/u_sq/g_lblk.blk_q_reg
```
