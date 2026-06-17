// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_blk \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     -GSD_SLOW_HALF=3 -GSD_FAST_HALF=1 -GSD_INIT_TICKS=20 \
//     --top-module virtio_blk src/virtio_blk.v src/sd_host.v \
//     src/axi_single_beat_master.v src/virtio_blk_tb.cpp
//   ./obj_dir_blk/Vvirtio_blk
//
// Drives virtio_blk with a behavioral AXI slave (sparse 64-bit word DDR) for
// the vring + guest buffers, and the behavioral SD card (sd_card_model.h) on
// the SD pins as the persistent backing store. Exercises the full path:
//   1. T_IN  (read):  SD sector -> block buffer -> guest DATA buffer
//   2. T_OUT (write): guest DATA buffer -> block buffer -> SD sector
//   3. round trip: write a sector, then read it back, bytes match
#include "Vvirtio_blk.h"
#include "verilated.h"
#include "sd_spi_card_model.h"
#include <cstdio>
#include <cstdint>
#include <unordered_map>

// ---- sparse 8-byte-word memory (device AXI address space) -----------------
static std::unordered_map<uint32_t, uint64_t> g_mem;
static uint64_t mem_rd64(uint32_t addr) {
    auto it = g_mem.find(addr >> 3);
    return it == g_mem.end() ? 0 : it->second;
}
static void mem_wr64(uint32_t addr, uint64_t data, uint8_t strb) {
    uint64_t w = mem_rd64(addr);
    for (int b = 0; b < 8; b++)
        if (strb & (1u << b)) { w &= ~(0xffULL << (b*8)); w |= (data & (0xffULL << (b*8))); }
    g_mem[addr >> 3] = w;
}
static void mem_wr_bytes(uint32_t addr, const uint8_t* p, int n) {
    for (int i = 0; i < n; i++) {
        uint32_t a = addr + i;
        uint64_t w = mem_rd64(a & ~7u);
        w &= ~(0xffULL << ((a & 7) * 8));
        w |= ((uint64_t)p[i]) << ((a & 7) * 8);
        g_mem[(a & ~7u) >> 3] = w;
    }
}
static uint8_t mem_rd_byte(uint32_t a) { return (uint8_t)(mem_rd64(a & ~7u) >> ((a & 7) * 8)); }
static void mem_wr16(uint32_t a, uint16_t v){ uint8_t b[2]={(uint8_t)v,(uint8_t)(v>>8)}; mem_wr_bytes(a,b,2);}
static uint16_t mem_rd16(uint32_t a){ return mem_rd_byte(a) | (mem_rd_byte(a+1)<<8); }
static uint32_t mem_rd32(uint32_t a){ uint32_t v=0; for(int i=0;i<4;i++) v|=mem_rd_byte(a+i)<<(8*i); return v;}

static Vvirtio_blk* dut;
static SpiSdCard card;
static bool s_rvalid = false; static uint64_t s_rdata = 0;
static bool s_bvalid = false;

static void tick() {
    dut->sd_miso = card.miso & 1;

    dut->m_axi_arready = 1; dut->m_axi_awready = 1; dut->m_axi_wready = 1;
    dut->m_axi_rvalid = s_rvalid; dut->m_axi_rdata = s_rdata;
    dut->m_axi_rresp = 0; dut->m_axi_rlast = 1; dut->m_axi_rid = 1;
    dut->m_axi_bvalid = s_bvalid; dut->m_axi_bresp = 0; dut->m_axi_bid = 1;
    dut->clock = 0; dut->eval();

    bool ar_hs = dut->m_axi_arvalid && dut->m_axi_arready;
    bool aw_hs = dut->m_axi_awvalid && dut->m_axi_awready;
    bool w_hs  = dut->m_axi_wvalid  && dut->m_axi_wready;
    bool r_hs  = dut->m_axi_rvalid  && dut->m_axi_rready;
    bool b_hs  = dut->m_axi_bvalid  && dut->m_axi_bready;
    uint32_t araddr = dut->m_axi_araddr, awaddr = dut->m_axi_awaddr;
    uint64_t wdata = dut->m_axi_wdata; uint8_t wstrb = dut->m_axi_wstrb;

    dut->clock = 1; dut->eval();

    if (r_hs) s_rvalid = false;
    if (ar_hs && !s_rvalid) { s_rdata = mem_rd64(araddr); s_rvalid = true; }
    if (b_hs) s_bvalid = false;
    if (aw_hs && w_hs && !s_bvalid) { mem_wr64(awaddr, wdata, wstrb); s_bvalid = true; }

    card.clock_edge(dut->sd_sck, dut->sd_cs_n, dut->sd_mosi);
}

// ---- vring layout (device addresses) --------------------------------------
static const uint32_t DESC = 0x2000, AVAIL = 0x3000, USED = 0x4000;
static const uint32_t HDR = 0x5000, DATA = 0x6000, STATUS = 0x7000;
static const uint16_t F_NEXT = 1, F_WRITE = 2;

static void set_desc(int i, uint64_t addr, uint32_t len, uint16_t flags, uint16_t next) {
    uint8_t b[16];
    for (int k=0;k<8;k++) b[k]=(uint8_t)(addr>>(8*k));
    for (int k=0;k<4;k++) b[8+k]=(uint8_t)(len>>(8*k));
    b[12]=(uint8_t)flags; b[13]=(uint8_t)(flags>>8);
    b[14]=(uint8_t)next;  b[15]=(uint8_t)(next>>8);
    mem_wr_bytes(DESC + i*16, b, 16);
}
static void set_blk_hdr(uint32_t type, uint64_t sector) {
    uint8_t b[16] = {0};
    for (int k=0;k<4;k++) b[k]=(uint8_t)(type>>(8*k));
    for (int k=0;k<8;k++) b[8+k]=(uint8_t)(sector>>(8*k));
    mem_wr_bytes(HDR, b, 16);
}

static int fails = 0;
static void check(bool ok, const char* m) { if(!ok){ printf("FAIL: %s\n",m); fails++; } }

static void run_request(uint16_t avail_idx, uint16_t expect_used_idx) {
    mem_wr16(AVAIL + 0, 0);
    mem_wr16(AVAIL + 4 + (avail_idx-1)*2, 0);      // avail.ring[slot] = head desc 0
    mem_wr16(AVAIL + 2, avail_idx);                // avail.idx (publish)
    dut->queue_notify_pulse = 1; dut->queue_notify_value = 0; tick();
    dut->queue_notify_pulse = 0;
    for (int i = 0; i < 3000000; i++) {            // budget covers SD init + xfer
        tick();
        if (mem_rd16(USED + 2) == expect_used_idx) return;
    }
    check(false, "request timed out (used.idx never advanced)");
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vvirtio_blk;

    dut->reset = 1; dut->device_status = 0; dut->queue_notify_pulse = 0;
    dut->queue_num = 0; dut->queue_ready = 0;
    dut->queue_desc = 0; dut->queue_driver = 0; dut->queue_device = 0;
    for (int i = 0; i < 8; i++) tick();
    dut->reset = 0;

    dut->queue_num = 8; dut->queue_ready = 1;
    dut->queue_desc = DESC; dut->queue_driver = AVAIL; dut->queue_device = USED;
    dut->device_status = 0x0f; // ACK|DRIVER|FEATURES_OK|DRIVER_OK
    for (int i = 0; i < 4; i++) tick();

    // ---- Test 1: READ (T_IN) SD sector 3 -> DATA buffer -------------------
    const uint32_t SECT = 3;
    std::array<uint8_t,512> patt;
    for (int i = 0; i < 512; i++) patt[i] = (uint8_t)(i*7 + 1);
    card.store[SECT] = patt;

    set_blk_hdr(0 /*T_IN*/, SECT);
    set_desc(0, HDR,    16,  F_NEXT,            1);
    set_desc(1, DATA,   512, F_NEXT | F_WRITE,  2);
    set_desc(2, STATUS, 1,   F_WRITE,           0);
    for (int i = 0; i < 512; i++) mem_wr_bytes(DATA + i, (const uint8_t*)"\0", 1);
    run_request(1, 1);

    bool data_ok = true;
    for (int i = 0; i < 512; i++) if (mem_rd_byte(DATA + i) != patt[i]) data_ok = false;
    check(data_ok, "read: DATA buffer == SD sector");
    check(mem_rd_byte(STATUS) == 0, "read: status == S_OK");
    check(mem_rd16(USED + 4) == 0, "read: used.ring[0].id == head desc 0");
    check(mem_rd32(USED + 8) == 513, "read: used.ring[0].len == 512+1");

    // ---- Test 2: WRITE (T_OUT) DATA buffer -> SD sector 5 -----------------
    const uint32_t SECT2 = 5;
    std::array<uint8_t,512> patt2;
    for (int i = 0; i < 512; i++) patt2[i] = (uint8_t)(i ^ 0xa5);
    mem_wr_bytes(DATA, patt2.data(), 512);
    set_blk_hdr(1 /*T_OUT*/, SECT2);
    set_desc(0, HDR,    16,  F_NEXT,   1);
    set_desc(1, DATA,   512, F_NEXT,   2);
    set_desc(2, STATUS, 1,   F_WRITE,  0);
    mem_wr_bytes(STATUS, (const uint8_t*)"\xff", 1);
    run_request(2, 2);

    check(card.store.count(SECT2) && card.store[SECT2] == patt2,
          "write: SD sector == DATA buffer");
    check(mem_rd_byte(STATUS) == 0, "write: status == S_OK");
    check(mem_rd32(USED + 8 + 8) == 1, "write: used.ring[1].len == 1 (status only)");

    // ---- Test 3: round trip -- write sector 9, read it back ---------------
    const uint32_t SECT3 = 9;
    std::array<uint8_t,512> patt3;
    for (int i = 0; i < 512; i++) patt3[i] = (uint8_t)(0x13 + i*5 + (i>>5));
    mem_wr_bytes(DATA, patt3.data(), 512);
    set_blk_hdr(1 /*T_OUT*/, SECT3);
    set_desc(0, HDR, 16, F_NEXT, 1); set_desc(1, DATA, 512, F_NEXT, 2); set_desc(2, STATUS, 1, F_WRITE, 0);
    run_request(3, 3);

    for (int i = 0; i < 512; i++) mem_wr_bytes(DATA + i, (const uint8_t*)"\0", 1);
    set_blk_hdr(0 /*T_IN*/, SECT3);
    set_desc(0, HDR, 16, F_NEXT, 1); set_desc(1, DATA, 512, F_NEXT | F_WRITE, 2); set_desc(2, STATUS, 1, F_WRITE, 0);
    run_request(4, 4);

    bool rt_ok = true;
    for (int i = 0; i < 512; i++) if (mem_rd_byte(DATA + i) != patt3[i]) rt_ok = false;
    check(rt_ok, "round trip: read-back == written");

    // ---- Test 4: multi-sector READ (4 contiguous sectors -> one CMD18) ----
    {
        const uint32_t MSECT = 40; const int N = 4;
        std::array<uint8_t,512> mp[N];
        for (int s = 0; s < N; s++) {
            for (int i = 0; i < 512; i++) mp[s][i] = (uint8_t)(s*53 + i*3 + 7);
            card.store[MSECT + s] = mp[s];
        }
        set_blk_hdr(0 /*T_IN*/, MSECT);
        set_desc(0, HDR,    16,    F_NEXT,           1);
        set_desc(1, DATA,   512*N, F_NEXT | F_WRITE, 2);
        set_desc(2, STATUS, 1,     F_WRITE,          0);
        for (int i = 0; i < 512*N; i++) mem_wr_bytes(DATA + i, (const uint8_t*)"\0", 1);
        run_request(5, 5);
        bool ok = true;
        for (int s = 0; s < N; s++)
            for (int i = 0; i < 512; i++)
                if (mem_rd_byte(DATA + s*512 + i) != mp[s][i]) ok = false;
        check(ok, "multi-read: 4 sectors == SD store");
        check(mem_rd_byte(STATUS) == 0, "multi-read: status == S_OK");
    }

    // ---- Test 5: multi-sector WRITE (4 contiguous sectors -> one CMD25) ----
    {
        const uint32_t MSECT = 60; const int N = 4;
        std::array<uint8_t,512> mp[N];
        for (int s = 0; s < N; s++)
            for (int i = 0; i < 512; i++) mp[s][i] = (uint8_t)(0xA0 ^ (s*7) ^ i);
        for (int s = 0; s < N; s++) mem_wr_bytes(DATA + s*512, mp[s].data(), 512);
        set_blk_hdr(1 /*T_OUT*/, MSECT);
        set_desc(0, HDR,    16,    F_NEXT,  1);
        set_desc(1, DATA,   512*N, F_NEXT,  2);
        set_desc(2, STATUS, 1,     F_WRITE, 0);
        run_request(6, 6);
        bool ok = true;
        for (int s = 0; s < N; s++)
            if (!card.store.count(MSECT + s) || card.store[MSECT + s] != mp[s]) ok = false;
        check(ok, "multi-write: 4 SD sectors == DATA buffer");
        check(mem_rd_byte(STATUS) == 0, "multi-write: status == S_OK");
    }

    printf("%s (%d failure(s))\n", fails ? "virtio_blk: FAIL" : "virtio_blk: PASS", fails);
    delete dut;
    return fails ? 1 : 0;
}
