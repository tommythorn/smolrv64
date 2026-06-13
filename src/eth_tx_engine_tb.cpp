// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_txeng \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     --top-module eth_tx_engine_loop_top eth_tx_engine_loop_top.v \
//     eth_tx_engine.v eth_mac_tx.v eth_mac_rx.v crc32_d8.v eth_tx_engine_tb.cpp
//   ./obj_dir_txeng/Veth_tx_engine_loop_top
//
// Writes a frame into eth_tx_engine over the ui domain, pulses send, and checks
// it transmits through eth_mac_tx and is received by eth_mac_rx with rx_good,
// and that busy clears. ui_clk and gmii_clk run at different rates to exercise
// the send/done toggle-synchronizer CDC.
#include "Veth_tx_engine_loop_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto* dut = new Veth_tx_engine_loop_top;

    std::vector<uint8_t> frame;
    for (int i = 0; i < 64; i++) frame.push_back((uint8_t)(0x20 + i));  // >60, no pad

    // ui_clk half-period 5 fine units, gmii_clk half-period 3 (async-ish).
    long t = 0;
    std::vector<uint8_t> rx;
    int rx_good = -1, busy_seen = 0, done_seen = 0;
    int ui_prev = 0, g_prev = 0;
    int wr_i = 0;      // frame-write progress (on ui rising edges)
    int phase = 0;     // 0=reset,1=writing,2=send,3=run
    long send_tick = -1;

    auto eval = [&]() { dut->eval(); };

    dut->ui_rst = 1; dut->gmii_rst = 1;
    dut->wr_en = 0; dut->send = 0; dut->send_len = 0; dut->wr_addr = 0; dut->wr_data = 0;

    for (t = 0; t < 200000; t++) {
        int ui = (t / 5) % 2;
        int g  = (t / 3) % 2;
        dut->ui_clk = ui;
        dut->gmii_clk = g;
        eval();

        int ui_rise = (ui && !ui_prev);
        int g_rise  = (g && !g_prev);
        ui_prev = ui; g_prev = g;

        if (ui_rise) {
            // sequence the ui-domain stimulus on ui rising edges
            if (phase == 0) {
                static int rstn = 0;
                if (++rstn >= 4) { dut->ui_rst = 0; phase = 1; }
            } else if (phase == 1) {
                if (wr_i < (int)frame.size()) {
                    dut->wr_en = 1; dut->wr_addr = wr_i; dut->wr_data = frame[wr_i];
                    wr_i++;
                } else {
                    dut->wr_en = 0;
                    dut->send = 1; dut->send_len = frame.size();
                    phase = 2; send_tick = t;
                }
            } else if (phase == 2) {
                dut->send = 0; phase = 3;
            } else {
                if (dut->busy) busy_seen = 1;
                if (busy_seen && !dut->busy) done_seen = 1;
            }
        }
        // release gmii reset shortly after ui reset
        if (phase >= 1) dut->gmii_rst = 0;

        if (g_rise && phase >= 2) {
            if (dut->rx_valid) rx.push_back(dut->rx_data);
            if (dut->rx_last) rx_good = dut->rx_good;
        }
        if (done_seen && rx_good >= 0) break;
    }

    int fails = 0;
    auto check = [&](bool ok, const char* m){ if(!ok){printf("FAIL: %s\n",m);fails++;} };
    check(rx.size() == frame.size(), "rx length == frame length (no pad, 64B)");
    if (rx.size() == frame.size())
        for (size_t i = 0; i < frame.size(); i++) check(rx[i] == frame[i], "rx byte");
    check(rx_good == 1, "rx_good");
    check(busy_seen == 1, "busy asserted");
    check(done_seen == 1, "busy cleared after TX");
    printf("rx %zu bytes, good=%d busy_seen=%d done=%d, %d failure(s)\n",
           rx.size(), rx_good, busy_seen, done_seen, fails);
    printf(fails == 0 ? "eth_tx_engine: PASS\n" : "eth_tx_engine: FAIL\n");
    delete dut;
    return fails ? 1 : 0;
}
