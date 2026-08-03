# Deferred Optimizations

Optimizations we have deliberately kicked down the road: correct behavior is in
place, but a faster or cleaner implementation is known and intentionally not done
yet. Each entry records *what*, *why deferred*, *where* in the code, and the
*expected benefit*, so we can pick them up without re-deriving the analysis.

This is for performance/quality work that is safe to defer. It is **not** a bug
list and **not** a place for missing features.

---

## cbo.zero cold-miss read-for-ownership

**What:** `cbo.zero` is implemented as a cacheable write of zeros that reuses the
ordinary store path. On a *cache hit* it zeros all 8 banks of the resident line
in a single cycle (per-bank write-enable mask + uniform zero data). On a *cold
miss*, however, it still fetches the 64-byte line from memory before overwriting
it with zeros — a wasted read-for-ownership, since the entire line is discarded.

**Why deferred:** The hit path is already optimal and is the common case; the
miss path is correct, just wasteful. Skipping the fill needs a new branch in the
cache FSM (jump straight from the miss resolution to a zeroed dirty install),
which is more surface area on a zero-margin-timing core than the first cut
warranted.

**Where:** `src/smolrv64.v` — decode at the `CBO.ZERO` branch; data-path hooks in
the combinational cache write block (`cache_req_zero` at the install and
`CACHE_HIT_WRITE` cases). The fill happens via `CACHE_FILL_REQ` →
`CACHE_FILL_LINE_INSTALL`; the optimization is to bypass `CACHE_FILL_REQ` when
`cache_req_zero` is set and install a zero line directly.

**Expected benefit:** Removes one full line fill (64-byte memory read) per cold
`cbo.zero`. Linux `clear_page()` issues 64 `cbo.zero` ops per 4 KiB page, so on
cold pages this is up to 64 avoidable line reads per page zeroed.

---

## Single-cycle unaligned load/store across line / page boundaries

**What:** Unaligned accesses — including those that cross a cache line or page
boundary — are intended to complete in a single cycle. Today the cross-8-byte
case is handled by the two-beat split path (`dmem_store2` / `S_DMEM_STORE2`),
which issues two sequential bank writes.

**Why deferred:** It is a design goal, not a regression; the current split path is
correct. Making it genuinely single-cycle is a datapath change (e.g. even/odd
bank organization with distinct per-half write data), not a localized fix.

**Where:** `src/smolrv64.v` — `S_STORE_COMMIT` split handling (`dmem_store_split`,
`dmem_store2_*`) and `S_DMEM_STORE2`; cache bank write port (`dcache_bank_wr_*`).

**Expected benefit:** One-cycle unaligned/cross-boundary stores; also the natural
substrate for a future store-pair extension (two registers → 128 bits to an
aligned address), which needs distinct even/odd write data the current single
shared `dcache_bank_wr_data` bus cannot provide in one cycle.

**Note:** `cbo.zero` deliberately does **not** depend on the split path; it relies
only on the per-bank write-enable mask, which survives this rework.

---

## Svpbmt NC store: read-for-ownership on the flush-around path

**What:** An NC/IO store is implemented as flush-around — fill the whole 64-byte
line, merge the store, write the whole line back to memory, invalidate. The fill
is a read-for-ownership: it pulls the line from DRAM only to overwrite it.

**Why deferred:** Reusing the existing fill + writeback engine is what makes the
NC bypass need no new datapath. The RFO is correct for the virtio use case (avail
and used rings live on separate cache lines, so no same-line CPU/DMA write race).

**Where:** `src/smolrv64.v` — `FILL_LINE_INSTALL` beat-7 NC-store branch
(`cache_wb_after_ncstore`) and `cache_finish_writeback_line`.

**Expected benefit:** A true uncached store path (write only the store's bytes to
DRAM via a direct doubleword write — BRAM `mem0/mem1`, or an AXI single-beat
write) removes the RFO *and* closes the same-line-DMA-clobber hole, making NC
strictly correct. This is Option B in `SVPBMT_PLAN.md`.
