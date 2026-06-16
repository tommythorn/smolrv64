// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_spi \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     -GSLOW_HALF=2 -GFAST_HALF=1 -GINIT_BYTES=4 \
//     --top-module sd_spi_host src/sd_spi_host.v src/sd_spi_host_tb.cpp
//   ./obj_dir_spi/Vsd_spi_host
//
// Drives sd_spi_host against the behavioral SPI-SD card (sd_spi_card_model.h):
//   1. init handshake completes (ready) + CSD capacity
//   2. CMD17 read of a preloaded sector -> block buffer matches
//   3. CMD24 write of the block buffer -> card storage matches
#include "Vsd_spi_host.h"
#include "verilated.h"
#include "sd_spi_card_model.h"
#include <cstdio>
#include <cstdint>

static Vsd_spi_host* dut;
static SpiSdCard card;
static bool saw_done = false, saw_error = false;

static void core_cycle() {
    dut->miso = card.miso & 1;
    dut->clock = 0; dut->eval();
    dut->clock = 1; dut->eval();
    if (dut->done)  saw_done = true;
    if (dut->error) saw_error = true;
    card.clock_edge(dut->sck, dut->cs_n, dut->mosi);
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
    dut = new Vsd_spi_host;
    dut->reset = 1; dut->req_valid = 0; dut->buf_we = 0; dut->miso = 1;
    dut->clock = 0; dut->eval();
    for (int i = 0; i < 8; i++) { dut->clock=0; dut->eval(); dut->clock=1; dut->eval(); }
    dut->reset = 0;

    int failures = 0;

    long n = 0;
    while (dut->ready == 0 && n++ < 5000000) core_cycle();
    if (!dut->ready) { printf("FAIL: init did not complete\n"); failures++; }
    else printf("ok: init complete (ready)\n");

    if (dut->ready) {
        uint32_t exp = (card.csd_csize + 1) << 10;
        if (dut->capacity_sectors != exp) { printf("FAIL: capacity %u != %u\n", dut->capacity_sectors, exp); failures++; }
        else printf("ok: CSD capacity = %u sectors (%u MiB)\n", exp, exp/2048);
    }

    if (dut->ready) {
        std::array<uint8_t,512> pat;
        for (int i = 0; i < 512; i++) pat[i] = (uint8_t)(0xA0 ^ i ^ (i >> 3));
        card.store[1234] = pat;
        if (!run_request(0, 1234, 5000000)) { printf("FAIL: read did not complete\n"); failures++; }
        else {
            int bad = 0;
            for (int w = 0; w < 64; w++) {
                dut->buf_addr = w; dut->eval();
                uint64_t word = dut->buf_rdata;
                for (int k = 0; k < 8; k++) {
                    uint8_t got = (word >> (k*8)) & 0xff, e = pat[w*8+k];
                    if (got != e && bad < 8) { printf("  read mismatch byte %d: got %02x exp %02x\n", w*8+k, got, e); bad++; }
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
        if (!run_request(1, 4321, 5000000)) { printf("FAIL: write did not complete\n"); failures++; }
        else if (card.store.find(4321) == card.store.end() || card.store[4321] != pat) { printf("FAIL: written sector != buffer\n"); failures++; }
        else printf("ok: CMD24 write sector 4321 matches\n");
    }

    printf("sd_spi_host: %s (%d failure(s))\n", failures ? "FAIL" : "PASS", failures);
    delete dut;
    return failures ? 1 : 0;
}
