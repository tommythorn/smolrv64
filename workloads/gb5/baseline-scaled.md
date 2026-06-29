# SmolRV64 Ubuntu 2GiB — Geekbench 5.4.1 (RISC-V), scores ÷10

Source: https://browser.geekbench.com/v5/cpu/24416408
Result flagged invalid (timer issue). Score column divided by 10; measured rates unchanged.

This is the first GB5 baseline (SmolRV64, ~2026-06-26 build). Track future runs against the
measured *rates* (unchanged by the ÷10); the score column is GB5's normalized number with the
10× fast-timer fudge removed.

| Subtest | SC score ÷10 | SC rate | MC score ÷10 | MC rate |
|---|---|---|---|---|
| **Overall** | 0.9 | — | 1.0 | — |
| Crypto | 0.1 | — | 0.1 | — |
| Integer | 1.0 | — | 1.2 | — |
| Floating Point | 0.0 | — | 0.0 | — |
| AES-XTS | 0.1 | 1.48 MB/sec | 0.1 | 1.76 MB/sec |
| Text Compression | 2.6 | 135.3 KB/sec | 2.7 | 138.6 KB/sec |
| Image Compression | 0.8 | 400.4 Kpixels/sec | 0.8 | 396.7 Kpixels/sec |
| Navigation | 3.0 | 85.5 KTE/sec | 3.1 | 87.1 KTE/sec |
| HTML5 | 1.1 | 13.0 KElements/sec | 1.1 | 12.7 KElements/sec |
| SQLite | 1.3 | 3.99 Krows/sec | 1.2 | 3.88 Krows/sec |
| PDF Rendering | 0.5 | 282.2 Kpixels/sec | 1.1 | 623.9 Kpixels/sec |
| Text Rendering | 0.6 | 1.78 KB/sec | 1.2 | 3.80 KB/sec |
| Clang | 1.5 | 120.6 lines/sec | 1.8 | 140.6 lines/sec |
| Camera | 0.3 | 0.04 images/sec | 0.3 | 0.04 images/sec |
| N-Body Physics | 1.4 | 17.8 Kpairs/sec | 1.4 | 18.0 Kpairs/sec |
| Rigid Body Physics | 1.1 | 69.1 FPS | 0.9 | 58.2 FPS |
| Gaussian Blur | 0.1 | 65.9 Kpixels/sec | 0.1 | 66.2 Kpixels/sec |
| Face Detection | 1.0 | 0.08 images/sec | 1.0 | 0.08 images/sec |
| Horizon Detection | 1.1 | 275.6 Kpixels/sec | 1.2 | 286.4 Kpixels/sec |
| Image Inpainting | 1.1 | 537.2 Kpixels/sec | 1.1 | 547.3 Kpixels/sec |
| HDR | 1.7 | 225.5 Kpixels/sec | 1.5 | 200.6 Kpixels/sec |
| Ray Tracing | 1.2 | 9.42 Kpixels/sec | 1.2 | 9.62 Kpixels/sec |
| Structure from Motion | 0.3 | 28.3 pixels/sec | 0.3 | 27.5 pixels/sec |
| Speech Recognition | 1.9 | 0.62 Words/sec | 1.8 | 0.57 Words/sec |
| Machine Learning | 0.0 | 0.01 images/sec | 0.0 | 0.01 images/sec |

## Read of the baseline (single-core profile)

- **Broad ~10x IPC sag** across all subtests (best are only ~1.5-3) = the CPI redirect bug
  (backend discards the frontend's run-ahead every retire). Fixing it lifts the whole table.
- **FP crater**: FP overall 0.0; Gaussian Blur 0.1, ML 0.0, Camera/SfM 0.3 -- ~10x below the
  integer subtests. A specific FP-pipe problem (no FP bypass, exposed FPU/FMA latency) on top
  of the general sag.
- **Crypto crater**: AES-XTS 0.1 (1.48 MB/sec) -- software table-lookup AES, no Zkne/Zknd.
- **No SMP scaling**: MC overall 1.0 vs SC 0.9 -- effectively one core of throughput.

Priority levers: (1) CPI redirect bug (uniform multiplier), (2) FP bypass + latency-hiding,
(3) branch prediction (YAGS+RAS) for branchy integer, (4) Zkne/Zknd AES if Crypto matters.
