// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_rxeng \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     --top-module eth_rx_engine_loop_top eth_rx_engine_loop_top.v \
//     eth_mac_tx.v eth_mac_rx.v eth_rx_engine.v crc32_d8.v eth_rx_engine_tb.cpp
//   ./obj_dir_rxeng/Veth_rx_engine_loop_top
//
// Transmit a frame through eth_mac_tx -> eth_mac_rx -> eth_rx_engine across
// independent clocks, then read it back out the engine's ui side and confirm
// the recovered payload (incl pad to 60) matches.
#include "Veth_rx_engine_loop_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto* dut = new Veth_rx_engine_loop_top;

    std::vector<uint8_t> frame;
    for (int i = 0; i < 42; i++) frame.push_back((uint8_t)(0x30 + i));
    uint8_t buf[2048] = {0};
    for (size_t i = 0; i < frame.size(); i++) buf[i] = frame[i];

    dut->gmii_rst = 1; dut->ui_rst = 1;
    dut->send = 0; dut->tx_frame_len = 0; dut->tx_rd_data = 0;
    dut->rx_rd_addr = 0; dut->rx_frame_ack = 0;

    long t = 0; int ui_prev = 0, g_prev = 0;
    int phase = 0, rst_n = 0;
    std::vector<uint8_t> got;
    int rx_len = -1, read_i = 0, done = 0, fails = 0;

    for (t = 0; t < 400000 && !done; t++) {
        int ui = (t / 5) % 2, g = (t / 3) % 2;
        dut->gmii_clk = g; dut->ui_clk = ui;
        dut->tx_rd_data = buf[dut->tx_rd_index & 0x7ff];
        if (rx_len > 0) dut->rx_rd_data;          // (rd_data is async)
        dut->eval();

        int g_rise = (g && !g_prev), ui_rise = (ui && !ui_prev);
        g_prev = g; ui_prev = ui;

        if (g_rise) {  // gmii-domain stimulus
            if (phase == 0) { if (++rst_n >= 4) { dut->gmii_rst = 0; phase = 1; } }
            else if (phase == 1) { dut->tx_frame_len = frame.size(); dut->send = 1; phase = 2; }
            else if (phase == 2) { dut->send = 0; phase = 3; }
        }
        if (phase >= 1) dut->ui_rst = 0;

        if (ui_rise && dut->rx_frame_valid && rx_len < 0) {
            rx_len = dut->rx_frame_len;            // frame arrived; start reading
            read_i = 0;
        }
        if (ui_rise && rx_len >= 0 && read_i <= rx_len) {
            if (read_i < rx_len) {
                dut->rx_rd_addr = read_i;
                dut->eval();                       // settle async read
                got.push_back(dut->rx_rd_data);
            }
            read_i++;
            if (read_i > rx_len) { dut->rx_frame_ack = 1; }
        } else if (ui_rise) {
            dut->rx_frame_ack = 0;
            if (rx_len >= 0 && (int)got.size() >= rx_len) done = 1;
        }
    }

    auto check = [&](bool ok, const char* m){ if(!ok){printf("FAIL: %s\n",m);fails++;} };
    check(rx_len == 60, "rx_frame_len == 60 (42 + pad)");
    check((int)got.size() == 60, "read 60 bytes");
    if (got.size() == 60) {
        for (int i = 0; i < 42; i++) check(got[i] == frame[i], "payload byte");
        for (int i = 42; i < 60; i++) check(got[i] == 0x00, "pad byte");
    }
    check(dut->rx_drop_count == 0, "no drops");
    printf("rx_len=%d got=%zu drops=%u, %d failure(s)\n",
           rx_len, got.size(), dut->rx_drop_count, fails);
    printf(fails == 0 ? "eth_rx_engine: PASS\n" : "eth_rx_engine: FAIL\n");
    delete dut;
    return fails ? 1 : 0;
}
