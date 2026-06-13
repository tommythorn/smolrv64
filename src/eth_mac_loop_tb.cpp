// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_ethloop \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     --top-module eth_loop_top eth_loop_top.v eth_mac_tx.v eth_mac_rx.v \
//     crc32_d8.v eth_mac_loop_tb.cpp
//   ./obj_dir_ethloop/Veth_loop_top
//
// Loopback test: send an L2 frame through eth_mac_tx, receive it via
// eth_mac_rx, and check the recovered payload (incl zero-pad to 60) matches
// and rx_good asserts. Then flip one wire bit and confirm rx_good deasserts.
#include "Veth_loop_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Veth_loop_top* dut = new Veth_loop_top;

    std::vector<uint8_t> frame;
    for (int i = 0; i < 42; i++) frame.push_back((uint8_t)(0x10 + i));
    uint8_t buf[2048] = {0};
    for (size_t i = 0; i < frame.size(); i++) buf[i] = frame[i];

    auto tick = [&]() {
        dut->clk = 0; dut->eval();
        dut->rd_data = buf[dut->rd_index & 0x7ff];
        dut->eval();
        dut->clk = 1; dut->eval();
    };

    dut->corrupt = 0;
    dut->rst_n = 0; dut->send = 0; dut->frame_len = 0; dut->rd_data = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 2; i++) tick();

    dut->frame_len = frame.size();
    dut->send = 1; tick();
    dut->send = 0;

    std::vector<uint8_t> rx;
    int rx_good = -1, done = 0;
    for (int c = 0; c < 5000 && !done; c++) {
        tick();
        if (dut->rx_valid) rx.push_back(dut->rx_data);
        if (dut->rx_last) { rx_good = dut->rx_good; done = 1; }
    }

    int fails = 0;
    auto check = [&](bool ok, const char* m) { if (!ok) { printf("FAIL: %s\n", m); fails++; } };

    // Expected payload = 60 bytes: 42 frame + 18 zero pad.
    check(rx.size() == 60, "rx payload length == 60");
    if (rx.size() == 60) {
        for (int i = 0; i < 42; i++) check(rx[i] == frame[i], "rx payload byte");
        for (int i = 42; i < 60; i++) check(rx[i] == 0x00, "rx pad byte");
    }
    check(rx_good == 1, "rx_good asserted for clean frame");
    printf("rx %zu bytes, good=%d, %d failure(s)\n", rx.size(), rx_good, fails);

    // Negative test: flip one bit on the wire mid-frame -> FCS must fail.
    dut->rst_n = 0; for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1; for (int i = 0; i < 2; i++) tick();
    dut->frame_len = frame.size(); dut->send = 1; tick(); dut->send = 0;
    int bad_good = -1; done = 0;
    for (int c = 0; c < 5000 && !done; c++) {
        dut->corrupt = (c == 30) ? 1 : 0;   // one wire byte, mid-data
        tick();
        if (dut->rx_last) { bad_good = dut->rx_good; done = 1; }
    }
    check(bad_good == 0, "rx_good deasserted for corrupted frame");
    printf("corrupted frame good=%d\n", bad_good);

    printf(fails == 0 ? "eth_mac_loop: PASS\n" : "eth_mac_loop: FAIL\n");
    delete dut;
    return fails ? 1 : 0;
}
