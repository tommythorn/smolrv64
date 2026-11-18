// Build & run (src/run-tb.sh does this):
//   verilator --cc --exe --build -Mdir obj_dir_txeng \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     --top-module eth_tx_engine_loop_top eth_tx_engine_loop_top.v \
//     eth_tx_engine.v eth_mac_tx.v eth_mac_rx.v crc32_d8.v eth_tx_engine_tb.cpp
//   ./obj_dir_txeng/Veth_tx_engine_loop_top
//
// eth_tx_engine through eth_mac_tx into eth_mac_rx, ui_clk and gmii_clk at unrelated rates:
//  1. a frame goes out whole, with a good FCS, and busy clears;
//  2. the gmii domain is reset -- its clock stopped for a while, as when the PHY's receive clock
//     stops for a link renegotiation -- at every gmii cycle of a frame's life, from before the
//     send reaches the gmii domain to after its done toggle: the frame still arrives whole and
//     good (twice at most: a reset in the inter-frame gap after its FCS sends it again) and busy
//     clears; nothing else arrives;
//  3. a reset with nothing pending sends nothing, and frames after every reset go out normally.
#include "Veth_tx_engine_loop_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

static Veth_tx_engine_loop_top* dut;
static long t = 0, gcnt = 0;
static bool g_run = true, ui_rose = false, g_rose = false;
static std::vector<uint8_t> rx_cur;
static std::vector<std::pair<std::vector<uint8_t>, int>> rx_frames;   // {payload, good}
static int fails = 0;

static void check(bool ok, const char* m, int k = -1) {
    if (!ok) { printf("FAIL: %s (offset %d)\n", m, k); fails++; }
}

// One fine time unit: ui_clk half-period 5, gmii_clk half-period 3 while it runs.
static void step() {
    int ui_prev = dut->ui_clk, g_prev = dut->gmii_clk;
    t++;
    if (g_run) gcnt++;
    dut->ui_clk = (t / 5) % 2;
    dut->gmii_clk = (gcnt / 3) % 2;
    dut->eval();
    ui_rose = dut->ui_clk && !ui_prev;
    g_rose = dut->gmii_clk && !g_prev;
    if (g_rose) {
        if (dut->rx_valid) rx_cur.push_back(dut->rx_data);
        if (dut->rx_last) { rx_frames.push_back({rx_cur, dut->rx_good}); rx_cur.clear(); }
    }
}
static void ui_edge() { do step(); while (!ui_rose); }
static void g_edge() { do step(); while (!g_rose); }

static void gmii_reset(int stopped) {    // asserted asynchronously, the clock then stops
    dut->gmii_rst = 1; dut->eval();
    rx_cur.clear();                      // eth_mac_rx forgets a frame it was receiving
    g_run = false;
    for (int i = 0; i < stopped; i++) step();
    g_run = true;
    for (int i = 0; i < 2; i++) g_edge();   // the shell's synchronizer: two cycles of clock
    dut->gmii_rst = 0;
}

static std::vector<uint8_t> make_frame(int n, int seed) {
    std::vector<uint8_t> f;
    for (int i = 0; i < n; i++) f.push_back((uint8_t)(seed * 37 + i * 11));
    return f;
}

static void write_and_send(const std::vector<uint8_t>& f) {
    for (size_t i = 0; i < f.size(); i++) {
        dut->wr_en = 1; dut->wr_addr = i; dut->wr_data = f[i]; ui_edge();
    }
    dut->wr_en = 0;
    dut->send = 1; dut->send_len = f.size(); ui_edge();
    dut->send = 0;
}

static bool wait_idle() {
    for (int i = 0; i < 20000; i++) { ui_edge(); if (!dut->busy) return true; }
    return false;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Veth_tx_engine_loop_top;
    dut->wr_en = 0; dut->send = 0; dut->send_len = 0; dut->wr_addr = 0; dut->wr_data = 0;
    dut->gmii_rst = 1;
    for (int i = 0; i < 4; i++) g_edge();
    dut->gmii_rst = 0;
    check(!dut->busy, "busy at power-up");

    // 1. one frame
    auto f = make_frame(64, 1);
    write_and_send(f);
    check(dut->busy, "busy after send");
    check(wait_idle(), "busy cleared after TX");
    for (int i = 0; i < 50; i++) g_edge();
    check(rx_frames.size() == 1 && rx_frames[0].first == f && rx_frames[0].second == 1, "frame 1 whole and good");

    // 2. a reset at every gmii cycle of a frame's life (a 64-byte frame is ~100 cycles on the wire)
    int dup = 0, frames = 1;
    for (int k = 0; k < 140; k++) {
        auto g = make_frame(60 + k % 7, k + 2);
        size_t before = rx_frames.size();
        write_and_send(g);
        for (int i = 0; i < k; i++) g_edge();
        gmii_reset(50 + k % 13);
        check(wait_idle(), "busy cleared after a reset", k);
        for (int i = 0; i < 50; i++) g_edge();
        size_t got = rx_frames.size() - before;
        check(got == 1 || got == 2, "the frame arrives once or twice", k);
        if (got == 2) dup++;
        for (size_t j = before; j < rx_frames.size(); j++)
            check(rx_frames[j].first == g && rx_frames[j].second == 1, "the frame whole and good", k);
        frames++;
    }

    // 3. a reset with nothing pending sends nothing; the next frame goes out as normal
    size_t before = rx_frames.size();
    gmii_reset(40);
    for (int i = 0; i < 300; i++) g_edge();
    check(rx_frames.size() == before && !dut->busy, "an idle reset sends nothing");
    auto h = make_frame(1514, 99);
    write_and_send(h);
    check(wait_idle(), "busy cleared after a full-size frame");
    for (int i = 0; i < 50; i++) g_edge();
    check(rx_frames.size() == before + 1 && rx_frames.back().first == h && rx_frames.back().second == 1,
          "a full-size frame after the resets");
    frames++;

    printf("%d frames sent through 141 gmii resets, %d sent twice (a reset after the FCS), %zu received\n",
           frames, dup, rx_frames.size());
    printf("eth_tx_engine_loop_top: %s (%d failure(s))\n", fails ? "FAIL" : "PASS", fails);
    delete dut;
    return fails ? 1 : 0;
}
