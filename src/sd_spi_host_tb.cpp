// Build & run:
//   verilator --cc --exe --build -Mdir obj_dir_spi \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
//     -GSLOW_HALF=3 -GFAST_HALF=4 -GINIT_BYTES=4   (halves >=3: miso_sync margin) \
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

static bool run_request(int write, uint32_t sector, int last, long budget) {
    saw_done = saw_error = false;
    dut->req_valid = 1; dut->req_write = write; dut->req_sector = sector; dut->req_last = last;
    long n = 0;
    while (dut->busy == 0 && n++ < budget) core_cycle();
    dut->req_valid = 0;
    while (!saw_done && !saw_error && n++ < budget) core_cycle();
    return saw_done && !saw_error;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vsd_spi_host;
    dut->reset = 1; dut->req_valid = 0; dut->req_last = 1; dut->buf_we = 0; dut->miso = 1;
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
        if (!run_request(0, 1234, 1, 5000000)) { printf("FAIL: read did not complete\n"); failures++; }
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
        if (!run_request(1, 4321, 1, 5000000)) { printf("FAIL: write did not complete\n"); failures++; }
        else if (card.store.find(4321) == card.store.end() || card.store[4321] != pat) { printf("FAIL: written sector != buffer\n"); failures++; }
        else printf("ok: CMD24 write sector 4321 matches\n");
    }

    // ---- Test 4: CMD18 multi-block read of 3 contiguous sectors -----------
    if (dut->ready) {
        std::array<uint8_t,512> p[3];
        for (int s = 0; s < 3; s++) {
            for (int i = 0; i < 512; i++) p[s][i] = (uint8_t)(s*37 + i*5 + 1);
            card.store[2000 + s] = p[s];
        }
        int bad = 0;
        for (int s = 0; s < 3; s++) {
            if (!run_request(0, 2000 + s, s == 2, 5000000)) { printf("FAIL: mb read blk %d\n", s); failures++; bad = -1; break; }
            for (int w = 0; w < 64; w++) {
                dut->buf_addr = w; dut->eval();
                uint64_t word = dut->buf_rdata;
                for (int k = 0; k < 8; k++)
                    if ((uint8_t)(word >> (k*8)) != p[s][w*8+k] && bad < 8) {
                        printf("  mb read mismatch s%d byte %d: got %02x exp %02x\n",
                               s, w*8+k, (uint8_t)(word >> (k*8)), p[s][w*8+k]); bad++;
                    }
            }
        }
        if (bad > 0) { printf("FAIL: multi-block read data\n"); failures++; }
        else if (bad == 0) printf("ok: CMD18 multi-block read (3 sectors) matches\n");
    }

    // ---- Test 5: CMD25 multi-block write of 3 contiguous sectors ----------
    if (dut->ready) {
        std::array<uint8_t,512> p[3];
        bool ok = true;
        for (int s = 0; s < 3 && ok; s++) {
            for (int i = 0; i < 512; i++) p[s][i] = (uint8_t)(0x80 + s*13 + i*3);
            dut->buf_we = 1;
            for (int w = 0; w < 64; w++) {
                uint64_t word = 0;
                for (int k = 0; k < 8; k++) word |= (uint64_t)p[s][w*8+k] << (k*8);
                dut->buf_addr = w; dut->buf_wdata = word; core_cycle();
            }
            dut->buf_we = 0;
            if (!run_request(1, 3000 + s, s == 2, 5000000)) { printf("FAIL: mb write blk %d\n", s); failures++; ok = false; }
        }
        if (ok) {
            int bad = 0;
            for (int s = 0; s < 3; s++)
                if (card.store.find(3000+s) == card.store.end() || card.store[3000+s] != p[s]) bad++;
            if (bad) { printf("FAIL: multi-block write data (%d sectors)\n", bad); failures++; }
            else printf("ok: CMD25 multi-block write (3 sectors) matches\n");
        }
    }

    // ---- Test 6: read CRC error is detected and retried (CMD17) -----------
    if (dut->ready) {
        std::array<uint8_t,512> pat;
        for (int i = 0; i < 512; i++) pat[i] = (uint8_t)(0x11 + i*7);
        card.store[5555] = pat;
        card.corrupt_reads = 1;   // first attempt returns a bit-flipped block
        if (!run_request(0, 5555, 1, 5000000)) { printf("FAIL: crc-retry read did not complete\n"); failures++; }
        else {
            int bad = 0;
            for (int w = 0; w < 64; w++) {
                dut->buf_addr = w; dut->eval();
                uint64_t word = dut->buf_rdata;
                for (int k = 0; k < 8; k++)
                    if ((uint8_t)(word >> (k*8)) != pat[w*8+k]) bad++;
            }
            if (bad) { printf("FAIL: crc-retry read data mismatch (%d bytes)\n", bad); failures++; }
            else if (!(dut->dbg_io & (1 << 10))) { printf("FAIL: dbg crc_err not set\n"); failures++; }
            else if (!(dut->dbg_io & (1 << 9)))  { printf("FAIL: dbg retried not set\n"); failures++; }
            else printf("ok: read CRC error detected, retried, data clean\n");
        }
    }

    // ---- Test 7: persistent read corruption errors out (no silent data) ---
    if (dut->ready) {
        card.corrupt_reads = 100;  // more than the 1+7 attempts
        if (run_request(0, 5555, 1, 50000000)) { printf("FAIL: persistently corrupt read reported success\n"); failures++; }
        else printf("ok: persistent CRC corruption -> error after retries\n");
        card.corrupt_reads = 0;
    }

    // ---- Test 8: mid-multi-block CRC error is retried invisibly -----------
    if (dut->ready) {
        std::array<uint8_t,512> p[3];
        for (int s = 0; s < 3; s++) {
            for (int i = 0; i < 512; i++) p[s][i] = (uint8_t)(0xC3 - i*11 + s*29);
            card.store[6000 + s] = p[s];
        }
        card.corrupt_reads = 1;   // hits the first streamed block of the run
        int bad = 0;
        for (int s = 0; s < 3; s++) {
            if (!run_request(0, 6000 + s, s == 2, 50000000)) { printf("FAIL: mb crc-retry blk %d\n", s); failures++; bad = -1; break; }
            for (int w = 0; w < 64; w++) {
                dut->buf_addr = w; dut->eval();
                uint64_t word = dut->buf_rdata;
                for (int k = 0; k < 8; k++)
                    if ((uint8_t)(word >> (k*8)) != p[s][w*8+k]) bad++;
            }
        }
        if (bad > 0) { printf("FAIL: mb crc-retry data mismatch (%d bytes)\n", bad); failures++; }
        else if (bad == 0) printf("ok: mid-multi-block CRC error retried, run completes clean\n");
    }

    // ---- Test 9: persistent corruption in a multi-block run errors out ----
    if (dut->ready) {
        card.corrupt_reads = 100;
        if (run_request(0, 6000, 0, 50000000)) { printf("FAIL: persistently corrupt mb read reported success\n"); failures++; }
        else printf("ok: persistent mb corruption -> error after retries\n");
        card.corrupt_reads = 0;
        if (!run_request(0, 6000, 1, 5000000)) { printf("FAIL: clean read after mb error failed\n"); failures++; }
        else printf("ok: clean read after mb error\n");
    }

    printf("sd_spi_host: %s (%d failure(s))\n", failures ? "FAIL" : "PASS", failures);
    delete dut;
    return failures ? 1 : 0;
}
