// Behavioral SPI-mode SD card for Verilator testbenches.
// Mode 0: master changes MOSI on the falling SCK edge, samples MISO on rising;
// the card (slave) samples MOSI on rising, updates MISO on falling. CS active-low.
// Feed it the SPI pins each core cycle via clock_edge(); read miso back.
//
// Supports single-block CMD17/CMD24 and multi-block CMD18/CMD25:
//   CMD18 streams {0xFE,512B,CRC} per block until CMD12 stops it.
//   CMD25 receives {0xFC,512B,CRC} per block until the 0xFD stop-tran token.
#pragma once
#include <cstdint>
#include <map>
#include <deque>
#include <array>
#include <vector>
#include <unistd.h>
#include <sys/types.h>

struct SpiSdCard {
    std::map<uint32_t, std::array<uint8_t,512>> store;
    int miso = 1;

    // Optional image-file backing: a fd opened O_RDWR by the harness. Sectors are
    // loaded lazily on first access (sparse: a hole / past-EOF reads back as zero) and
    // written through on store, so the disk persists across the run. fd<0 -> pure RAM.
    int img_fd = -1;
    void attach_image(int fd) {
        img_fd = fd;
        // Advertise the image's size via the CSD-v2 C_SIZE: capacity = (csd_csize+1)*1024
        // sectors of 512B. Round the file down to the 512KB CSD granularity.
        off_t sz = lseek(fd, 0, SEEK_END);
        if (sz >= (off_t)512 * 1024) csd_csize = (uint32_t)(sz / (512 * 1024)) - 1;
    }
    std::array<uint8_t,512>& sect(uint32_t s) {
        auto it = store.find(s);
        if (it != store.end()) return it->second;
        auto& blk = store[s];                       // default-constructs to zero
        if (img_fd >= 0) {
            ssize_t got = pread(img_fd, blk.data(), 512, (off_t)s * 512);
            (void)got;                              // short/hole read -> remainder stays zero
        }
        return blk;
    }
    void writeback(uint32_t s) {
        if (img_fd >= 0) (void)!pwrite(img_fd, store[s].data(), 512, (off_t)s * 512);
    }

    // bit/byte framing
    int prev_sck = 0;
    int bitpos = 0;
    uint8_t in_byte = 0;
    uint8_t out_byte = 0xff;
    std::deque<uint8_t> txq;

    // protocol state
    enum { IDLE, CMD, WRITE_RECV } st = IDLE;
    int cmd = 0, cmd_pos = 0;
    uint32_t arg = 0;
    bool card_idle = true;     // SPI R1 idle bit until ACMD41 completes
    int acmd41_tries = 0;
    uint32_t csd_csize = 8191; // (8191+1)*1024 = 8388608 sectors (4 GB)
    // write-data receive
    int wr_state = 0, wr_idx = 0;
    uint32_t wr_sector = 0;
    std::array<uint8_t,512> wbuf{};
    bool high_capacity = true; // CCS=1 (block addressing)
    // multi-block read (CMD18 .. CMD12) / write (CMD25 .. 0xFD)
    bool mread_active = false;
    uint32_t mread_sector = 0;
    bool wr_multi = false;

    void reset_transfer() { bitpos = 0; in_byte = 0; out_byte = 0xff; miso = 1;
                            st = IDLE; cmd_pos = 0; txq.clear(); mread_active = false; }

    void push_block(const std::array<uint8_t,512>& d) {
        txq.push_back(0xFE);
        for (int i = 0; i < 512; i++) txq.push_back(d[i]);
        txq.push_back(0xff); txq.push_back(0xff);   // 2 CRC bytes
    }

    void process_command() {
        uint8_t r1 = card_idle ? 0x01 : 0x00;
        uint32_t sector = high_capacity ? arg : (arg >> 9);
        switch (cmd) {
        case 0:  txq.push_back(0x01); break;                       // GO_IDLE
        case 8:  txq.push_back(0x01);                              // R7 (idle)
                 txq.push_back(0x00); txq.push_back(0x00);
                 txq.push_back(0x01); txq.push_back(0xAA); break;
        case 55: txq.push_back(r1); break;
        case 41: if (++acmd41_tries >= 2) card_idle = false;
                 txq.push_back(card_idle ? 0x01 : 0x00); break;
        case 58: txq.push_back(0x00);                              // R3 OCR, CCS=1
                 txq.push_back(0xC0); txq.push_back(0xFF);
                 txq.push_back(0x80); txq.push_back(0x00); break;
        case 9: {                                                  // SEND_CSD (v2)
                 txq.push_back(0x00);
                 uint8_t c[16] = {0};
                 c[0] = 0x40;                                      // CSD_STRUCTURE=01
                 c[7] = (csd_csize >> 16) & 0x3f;
                 c[8] = (csd_csize >> 8) & 0xff;
                 c[9] = csd_csize & 0xff;
                 txq.push_back(0xFE);
                 for (int i = 0; i < 16; i++) txq.push_back(c[i]);
                 txq.push_back(0xff); txq.push_back(0xff);
                 break; }
        case 17: txq.push_back(0x00); push_block(sect(sector)); break;
        case 18: txq.push_back(0x00);                              // READ_MULTIPLE_BLOCK
                 mread_active = true; mread_sector = sector;
                 push_block(sect(mread_sector++)); break;
        case 12: mread_active = false; break;   // STOP_TRANSMISSION: stop refilling;
                 // the already-queued in-flight block drains, then the bus idles (0xFF) —
                 // modelling that a real card finishes the current block before going idle.
        case 24: txq.push_back(0x00); wr_sector = sector;
                 wr_multi = false; st = WRITE_RECV; wr_state = 0; return;
        case 25: txq.push_back(0x00); wr_sector = sector;          // WRITE_MULTIPLE_BLOCK
                 wr_multi = true; st = WRITE_RECV; wr_state = 0; return;
        default: txq.push_back(0x05); break;                      // illegal command
        }
        st = IDLE;
    }

    void recv_write_byte(uint8_t b) {
        if (wr_state == 0) {                 // waiting for a token
            if (b == 0xFE || b == 0xFC) { wr_state = 1; wr_idx = 0; }  // block start
            else if (b == 0xFD) {            // stop-tran token (multi-write end)
                txq.push_back(0x00);         // busy (one low byte)
                txq.push_back(0xff);         // released
                wr_multi = false; st = IDLE;
            }
        } else if (wr_state == 1) {          // 512 data bytes
            wbuf[wr_idx++] = b;
            if (wr_idx == 512) wr_state = 2;
        } else if (wr_state == 2) {          // CRC byte 1
            wr_state = 3;
        } else {                              // CRC byte 2 -> store + respond
            uint32_t s = wr_sector++; store[s] = wbuf; writeback(s);
            txq.push_back(0x05);             // data-response: accepted
            txq.push_back(0x00);             // busy (one low byte)
            txq.push_back(0xff);             // released
            if (wr_multi) wr_state = 0;      // await next 0xFC block or 0xFD stop
            else st = IDLE;
        }
    }

    void process_byte(uint8_t b) {
        if (st == WRITE_RECV) { recv_write_byte(b); return; }
        if (st == IDLE) {
            if ((b & 0xc0) == 0x40) { cmd = b & 0x3f; cmd_pos = 1; arg = 0; st = CMD; }
            return;
        }
        // st == CMD: collect arg (4 bytes) then CRC (1 byte)
        if (cmd_pos >= 1 && cmd_pos <= 4) arg = (arg << 8) | b;
        cmd_pos++;
        if (cmd_pos == 6) process_command();
    }

    uint8_t next_out() {
        if (txq.empty() && mread_active)       // keep the multi-read stream flowing
            push_block(sect(mread_sector++));
        if (txq.empty()) return 0xff;
        uint8_t b = txq.front(); txq.pop_front(); return b;
    }

    void clock_edge(int sck, int cs_n, int mosi) {
        if (cs_n) { reset_transfer(); prev_sck = sck; return; }
        if (sck && !prev_sck) {                       // rising: sample MOSI
            in_byte = (in_byte << 1) | (mosi & 1);
            if (++bitpos == 8) {
                process_byte(in_byte);
                out_byte = next_out();
                in_byte = 0; bitpos = 0;
                miso = (out_byte >> 7) & 1;
            }
        } else if (!sck && prev_sck) {                // falling: advance MISO
            miso = (out_byte >> (7 - bitpos)) & 1;
        }
        prev_sck = sck;
    }
};
