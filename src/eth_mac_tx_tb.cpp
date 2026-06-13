// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_ethtx \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     --top-module eth_mac_tx eth_mac_tx.v crc32_d8.v eth_mac_tx_tb.cpp
//   ./obj_dir_ethtx/Veth_mac_tx
//
// Verilator testbench for eth_mac_tx: drive one L2 frame, capture the GMII
// byte stream, and check preamble/SFD, payload, zero-pad to 60, and the 4 FCS
// bytes against an independent software Ethernet CRC32. A pass means the on-
// wire framing matches what a real host expects.
#include "Veth_mac_tx.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

static Veth_mac_tx* dut;

// Standard Ethernet FCS: reflected CRC-32 (poly 0xEDB88320), init/xor all-ones.
static uint32_t eth_fcs(const std::vector<uint8_t>& d) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint8_t b : d) {
        crc ^= b;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
    }
    return ~crc;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Veth_mac_tx;

    // Test L2 frame: 42 bytes (forces zero-pad to 60).
    std::vector<uint8_t> frame;
    for (int i = 0; i < 42; i++) frame.push_back((uint8_t)(0x10 + i));
    uint8_t buf[2048] = {0};
    for (size_t i = 0; i < frame.size(); i++) buf[i] = frame[i];

    auto eval = [&]() { dut->eval(); };
    auto tick = [&]() {
        dut->clk = 0; eval();
        dut->rd_data = buf[dut->rd_index & 0x7ff];  // combinational buffer read
        eval();
        dut->clk = 1; eval();
    };

    // Reset
    dut->rst_n = 0; dut->send = 0; dut->frame_len = 0; dut->rd_data = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    for (int i = 0; i < 2; i++) tick();

    // Kick off one frame
    dut->frame_len = frame.size();
    dut->send = 1; tick();
    dut->send = 0;

    // Capture the GMII stream while tx_en is high
    std::vector<uint8_t> wire;
    int idle_after = 0;
    for (int c = 0; c < 5000; c++) {
        tick();
        if (dut->gmii_tx_en) wire.push_back(dut->gmii_txd);
        else if (!wire.empty() && ++idle_after > 4) break;
    }

    // Expected: 8 preamble + 60 data + 4 FCS = 72 bytes
    int fails = 0;
    auto check = [&](bool ok, const char* msg) {
        if (!ok) { printf("FAIL: %s\n", msg); fails++; }
    };
    check(wire.size() == 72, "frame length (expect 72 GMII bytes)");
    if (wire.size() >= 72) {
        for (int i = 0; i < 7; i++) check(wire[i] == 0x55, "preamble 0x55");
        check(wire[7] == 0xD5, "SFD 0xD5");
        for (int i = 0; i < 42; i++) check(wire[8 + i] == frame[i], "payload byte");
        for (int i = 42; i < 60; i++) check(wire[8 + i] == 0x00, "zero pad");

        std::vector<uint8_t> data60(wire.begin() + 8, wire.begin() + 68);
        uint32_t fcs = eth_fcs(data60);
        uint8_t exp[4] = { (uint8_t)(fcs), (uint8_t)(fcs >> 8),
                           (uint8_t)(fcs >> 16), (uint8_t)(fcs >> 24) };
        for (int i = 0; i < 4; i++) {
            check(wire[68 + i] == exp[i], "FCS byte");
            if (wire[68 + i] != exp[i])
                printf("  FCS[%d] hw=%02x sw=%02x\n", i, wire[68 + i], exp[i]);
        }
    }

    printf("captured %zu GMII bytes, %d failure(s)\n", wire.size(), fails);
    printf(fails == 0 ? "eth_mac_tx: PASS\n" : "eth_mac_tx: FAIL\n");
    delete dut;
    return fails ? 1 : 0;
}
