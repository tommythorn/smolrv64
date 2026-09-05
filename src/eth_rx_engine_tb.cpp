// eth_rx_engine bench: frames through eth_mac_tx -> eth_mac_rx -> eth_rx_engine across
// independent clocks, read back out the engine's ui side (registered read) and checked
// byte for byte.  Run by ooo2/run-ooo2-ethrx-tb.sh (the gate picks that up).
//
//   A  one frame, padded to 60, read back                         (the original case)
//   B  SLOTS frames back to back, none acked until all are held   (a burst is absorbed)
//   C  a frame at a full ring is declined WHOLE and counted; the ring wraps after acks
//   D  the ack lands MID-frame at a full ring: still declined whole -- this is the
//      truncation the single-buffered engine shipped (2026-09-05): a tail delivered as a
//      good frame, one kernel rx_dropped per retransmitted NFS segment
//   E  an FCS-bad frame is counted, not delivered
//   F  a frame longer than a slot is counted, not delivered (built at -GSLOT_BYTES=1024)
#include "Veth_rx_engine_loop_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

static Veth_rx_engine_loop_top* dut;
static uint8_t txbuf[2048];
static long t = 0;
static int ui_prev = 0, g_prev = 0, g_rise = 0, ui_rise = 0;
static int fails = 0;
#define CHECK(ok, ...) do { if (!(ok)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

// one tick: ui period 10 ticks, gmii period 6 ticks (independent, incommensurate enough)
static void tick() {
    t++;
    int ui = (t / 5) % 2, g = (t / 3) % 2;
    dut->gmii_clk = g; dut->ui_clk = ui;
    dut->tx_rd_data = txbuf[dut->tx_rd_index & 0x7ff];
    dut->eval();
    g_rise = (g && !g_prev); ui_rise = (ui && !ui_prev);
    g_prev = g; ui_prev = ui;
}
static void ticks(int n) { while (n--) tick(); }
static void gmii_cycles(int n) { while (n) { tick(); if (g_rise) n--; } }
static void ui_cycles(int n)   { while (n) { tick(); if (ui_rise) n--; } }

static void fill(int id, int len) { for (int i = 0; i < len; i++) txbuf[i] = (uint8_t)(id * 37 + i); }

// pulse send for one gmii cycle; returns once the MAC has started (busy)
static void send_frame(int id, int len) {
    while (dut->tx_busy) tick();
    fill(id, len);
    dut->tx_frame_len = len;
    dut->send = 1; gmii_cycles(1);
    dut->send = 0;
    while (!dut->tx_busy) tick();
}
static void wait_tx_done() { while (dut->tx_busy) tick(); gmii_cycles(8); }   // + the deframer's tail

// read the held frame through the registered port: address at one ui edge, data at the next
static std::vector<uint8_t> read_frame(int len) {
    std::vector<uint8_t> got;
    for (int i = 0; i < len; i++) {
        dut->rx_rd_addr = i;
        ui_cycles(1);
        got.push_back(dut->rx_rd_data);
    }
    return got;
}
static void ack() { dut->rx_frame_ack = 1; ui_cycles(1); dut->rx_frame_ack = 0; ui_cycles(1); }

static void expect_frame(int id, int len, const char* what) {
    int budget = 200000;
    while (!dut->rx_frame_valid && budget--) tick();
    CHECK(dut->rx_frame_valid, "%s: frame %d not delivered", what, id);
    if (!dut->rx_frame_valid) return;
    int explen = len < 60 ? 60 : len;
    CHECK(dut->rx_frame_len == explen, "%s: frame %d len %d, expected %d", what, id, dut->rx_frame_len, explen);
    auto got = read_frame(dut->rx_frame_len);
    int bad = 0;
    for (int i = 0; i < (int)got.size(); i++) {
        uint8_t e = i < len ? (uint8_t)(id * 37 + i) : 0;
        if (got[i] != e) bad++;
    }
    CHECK(bad == 0, "%s: frame %d has %d wrong bytes", what, id, bad);
    ack();
}
static void expect_no_frame(const char* what) {
    ui_cycles(50);
    CHECK(!dut->rx_frame_valid, "%s: a frame is held that should not be", what);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Veth_rx_engine_loop_top;
    const int SLOTS = 8;
    dut->gmii_rst = 1; dut->ui_rst = 1;
    dut->send = 0; dut->tx_frame_len = 0; dut->tx_rd_data = 0; dut->corrupt = 0;
    dut->rx_rd_addr = 0; dut->rx_frame_ack = 0;
    ticks(60);
    dut->gmii_rst = 0; dut->ui_rst = 0;
    ticks(60);

    // ---- A: one short frame, padded to 60 ----
    send_frame(1, 42); wait_tx_done();
    expect_frame(1, 42, "A");
    CHECK(dut->rx_drop_count == 0 && dut->rx_bad_count == 0 && dut->rx_oflow_count == 0, "A: counts not zero");

    // ---- B: SLOTS frames back to back, held until all have landed ----
    for (int i = 0; i < SLOTS; i++) send_frame(10 + i, 64 + 9 * i);
    wait_tx_done();
    for (int i = 0; i < SLOTS; i++) expect_frame(10 + i, 64 + 9 * i, "B");
    CHECK(dut->rx_drop_count == 0, "B: busy drops %u, expected 0", dut->rx_drop_count);
    expect_no_frame("B");

    // ---- C: SLOTS+1 frames without an ack: the last is declined whole; the ring wraps ----
    for (int i = 0; i < SLOTS + 1; i++) send_frame(20 + i, 100);
    wait_tx_done();
    CHECK(dut->rx_drop_count == 1, "C: busy drops %u, expected 1", dut->rx_drop_count);
    for (int i = 0; i < SLOTS; i++) expect_frame(20 + i, 100, "C");
    expect_no_frame("C");                                  // the ninth left no tail behind
    send_frame(29, 77); wait_tx_done();                    // the ring is free again
    expect_frame(29, 77, "C wrap");

    // ---- D: the ack lands MID-frame at a full ring ----
    for (int i = 0; i < SLOTS; i++) send_frame(30 + i, 100);
    wait_tx_done();
    send_frame(38, 400);                                   // starts at a full ring: declined
    gmii_cycles(150);                                      // ...and while its bytes stream in,
    ack();                                                 // the backend releases a slot
    wait_tx_done();
    CHECK(dut->rx_drop_count == 2, "D: busy drops %u, expected 2", dut->rx_drop_count);
    for (int i = 1; i < SLOTS; i++) expect_frame(30 + i, 100, "D");
    expect_no_frame("D");                                  // no truncated tail of frame 38
    send_frame(39, 120); wait_tx_done();
    expect_frame(39, 120, "D next");                       // the freed slot takes a whole frame

    // ---- E: an FCS-bad frame ----
    send_frame(40, 200);
    gmii_cycles(40); dut->corrupt = 1; gmii_cycles(1); dut->corrupt = 0;
    wait_tx_done();
    CHECK(dut->rx_bad_count == 1, "E: bad %u, expected 1", dut->rx_bad_count);
    expect_no_frame("E");
    send_frame(41, 200); wait_tx_done();
    expect_frame(41, 200, "E next");

    // ---- F: a frame longer than a slot. The MAC's longest frame (2047) fits the shipping
    // 2048-byte slot, so the runner builds this bench with -GSLOT_BYTES=1024 and 1500 overflows.
    send_frame(42, 1500); wait_tx_done();
    CHECK(dut->rx_oflow_count == 1, "F: oflow %u, expected 1", dut->rx_oflow_count);
    expect_no_frame("F");
    send_frame(43, 90); wait_tx_done();
    expect_frame(43, 90, "F next");

    printf("drops=%u bad=%u oflow=%u, %d failure(s)\n",
           dut->rx_drop_count, dut->rx_bad_count, dut->rx_oflow_count, fails);
    printf(fails == 0 ? "eth_rx_engine: PASS\n" : "eth_rx_engine: FAIL\n");
    delete dut;
    return fails ? 1 : 0;
}
