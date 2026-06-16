// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_sd \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     -GSLOW_HALF=3 -GFAST_HALF=1 -GINIT_TICKS=20 \
//     --top-module sd_host src/sd_host.v src/sd_host_tb.cpp
//   ./obj_dir_sd/Vsd_host
//
// Drives sd_host against the behavioral SD card (sd_card_model.h). Exercises:
//   1. init handshake completes (ready)
//   2. CMD17 read of a preloaded sector -> block buffer matches
//   3. CMD24 write of the block buffer -> card storage matches
#include "Vsd_host.h"
#include "verilated.h"
#include "sd_card_model.h"
#include <cstdio>
#include <cstdint>

static Vsd_host* dut;
static SDCard card;
static bool saw_done = false, saw_error = false;

static void core_cycle() {
    dut->cmd_i = card.cmd_out & 1;
    dut->dat_i = (card.dat_out & 1) | 0xE;
    dut->clock = 0; dut->eval();
    dut->clock = 1; dut->eval();
    if (dut->done)  saw_done = true;
    if (dut->error) saw_error = true;
    card.clock_edge(dut->sd_clk, dut->cmd_t == 0, dut->cmd_o & 1,
                    (dut->dat_t & 1) == 0, dut->dat_o & 1);
}

static bool run_request(int write, uint32_t sector, long budget) {
    saw_done = saw_error = false;
    dut->req_valid = 1; dut->req_write = write; dut->req_sector = sector;
    long n = 0;
    while (dut->busy == 0 && n++ < budget) core_cycle();
    dut->req_valid = 0;
    while (!saw_done && !saw_error && n++ < budget) core_cycle();
    return saw_done && !saw_error;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vsd_host;
    dut->reset = 1; dut->req_valid = 0; dut->buf_we = 0;
    dut->clock = 0; dut->eval();
    for (int i = 0; i < 8; i++) { dut->clock=0; dut->eval(); dut->clock=1; dut->eval(); }
    dut->reset = 0;
    card.prev_sd = dut->sd_clk;

    int failures = 0;

    long n = 0;
    while (dut->ready == 0 && n++ < 2000000) core_cycle();
    if (!dut->ready) { printf("FAIL: init handshake did not complete\n"); failures++; }
    else printf("ok: init handshake complete (ready)\n");

    if (dut->ready) {
        uint32_t exp = (card.csd_csize + 1) << 10;
        if (dut->capacity_sectors != exp) {
            printf("FAIL: capacity %u != expected %u\n", dut->capacity_sectors, exp); failures++;
        } else printf("ok: CSD capacity = %u sectors (%u MiB)\n", exp, exp/2048);
    }

    if (dut->ready) {
        std::array<uint8_t,512> pat;
        for (int i = 0; i < 512; i++) pat[i] = (uint8_t)(0xA0 ^ i ^ (i >> 3));
        card.store[1234] = pat;
        if (!run_request(0, 1234, 5000000)) {
            printf("FAIL: read request did not complete cleanly\n"); failures++;
        } else {
            int bad = 0;
            for (int w = 0; w < 64; w++) {
                dut->buf_addr = w; dut->eval();
                uint64_t word = dut->buf_rdata;
                for (int k = 0; k < 8; k++) {
                    uint8_t got = (word >> (k*8)) & 0xff, exp = pat[w*8+k];
                    if (got != exp && bad < 8) { printf("  read mismatch byte %d: got %02x exp %02x\n", w*8+k, got, exp); bad++; }
                }
            }
            if (bad) { printf("FAIL: read data mismatch\n"); failures++; }
            else printf("ok: CMD17 read sector 1234 matches\n");
        }
    }

    if (dut->ready) {
        std::array<uint8_t,512> pat;
        for (int i = 0; i < 512; i++) pat[i] = (uint8_t)(0x5C + i*3 + (i>>4));
        dut->buf_we = 1;
        for (int w = 0; w < 64; w++) {
            uint64_t word = 0;
            for (int k = 0; k < 8; k++) word |= (uint64_t)pat[w*8+k] << (k*8);
            dut->buf_addr = w; dut->buf_wdata = word; core_cycle();
        }
        dut->buf_we = 0;
        if (!run_request(1, 4321, 5000000)) {
            printf("FAIL: write request did not complete cleanly\n"); failures++;
        } else if (card.store.find(4321) == card.store.end() || card.store[4321] != pat) {
            printf("FAIL: written sector 4321 does not match buffer\n"); failures++;
        } else printf("ok: CMD24 write sector 4321 matches\n");
    }

    printf("sd_host: %s (%d failure(s))\n", failures ? "FAIL" : "PASS", failures);
    delete dut;
    return failures ? 1 : 0;
}
