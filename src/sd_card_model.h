// Behavioral native-mode SD card for Verilator testbenches (DUT-agnostic).
//
// Feed it the SD pins each core cycle via clock_edge(); it detects SD-clock
// edges itself. Present cmd_out/dat_out back to the DUT's cmd_i/dat_i inputs.
#pragma once
#include <cstdint>
#include <map>
#include <vector>
#include <array>

struct SDCard {
    std::map<uint32_t, std::array<uint8_t,512>> store;

    int cmd_out = 1;          // present on DUT cmd_i (1 = idle/released)
    int dat_out = 1;          // present on DUT dat_i[0]
    int prev_sd = 0;

    enum { RX_CMD, RESP_DELAY, TX_RESP, RD_DRIVE, WR_RECV, WR_DRIVE } st = RX_CMD;

    uint64_t cmd_sh = 0;
    int      cmd_bits = -1;
    int      last_cmd = 0;
    uint32_t last_arg = 0;
    bool     app_cmd = false;
    int      acmd41_polls = 0;
    uint16_t rca = 0x0001;
    uint32_t csd_csize = 8191;   // CSD v2 C_SIZE -> (8191+1)*1024 = 8388608 sectors (4 GB)

    std::vector<int> resp;  int resp_i = 0;  int delay = 0;
    std::vector<int> txbits; int tx_i = 0;
    int rx_left = 0, rx_acc = 0, rx_bitcnt = 0, rx_bytecnt = 0;
    std::array<uint8_t,512> wbuf{};
    uint32_t cur_sector = 0;

    static uint16_t crc16(const uint8_t* d, int n) {
        uint16_t c = 0;
        for (int i = 0; i < n; i++)
            for (int b = 7; b >= 0; b--) {
                int bit = (d[i] >> b) & 1;
                int inv = bit ^ ((c >> 15) & 1);
                c = (c << 1) & 0xffff;
                if (inv) c ^= 0x1021;
            }
        return c;
    }

    void push_resp48(int idx, uint32_t content) {
        uint64_t f = ((uint64_t)(idx & 0x3f) << 40) | ((uint64_t)content << 8) | 1;
        resp.clear();
        for (int i = 47; i >= 0; i--) resp.push_back((f >> i) & 1);
        resp_i = 0;
    }
    void push_resp136() {
        resp.clear();
        resp.push_back(0); resp.push_back(0);
        for (int i = 0; i < 134; i++) resp.push_back((i * 7 + 3) & 1);
        resp_i = 0;
    }
    // R2 carrying a CSD v2.0: CSD_STRUCTURE=01, C_SIZE at CSD[69:48].
    void push_csd(uint32_t c_size) {
        resp.clear();
        resp.push_back(0); resp.push_back(0);                 // start, dir
        for (int i = 0; i < 6; i++) resp.push_back(1);        // reserved
        for (int n = 127; n >= 1; n--) {                      // CSD[127..1], MSB first
            int bit = 0;
            if (n == 126) bit = 1;                            // CSD_STRUCTURE = 01
            else if (n >= 48 && n <= 69) bit = (c_size >> (n - 48)) & 1;
            resp.push_back(bit);
        }
        resp.push_back(1);                                    // end bit
        resp_i = 0;
    }

    void begin_read_block(uint32_t sector) {
        auto& sec = store[sector];
        txbits.clear();
        for (int g = 0; g < 4; g++) txbits.push_back(1);   // gap
        txbits.push_back(0);                               // start bit
        for (int i = 0; i < 512; i++)
            for (int b = 7; b >= 0; b--) txbits.push_back((sec[i] >> b) & 1);
        uint16_t c = crc16(sec.data(), 512);
        for (int b = 15; b >= 0; b--) txbits.push_back((c >> b) & 1);
        txbits.push_back(1);
        tx_i = 0; st = RD_DRIVE;
    }
    void begin_write_block(uint32_t sector) {
        cur_sector = sector;
        rx_left = -1; rx_acc = rx_bitcnt = rx_bytecnt = 0;
        st = WR_RECV;
    }

    void decode_command() {
        int idx = (cmd_sh >> 40) & 0x3f;
        uint32_t arg = (cmd_sh >> 8) & 0xffffffff;
        last_cmd = idx; last_arg = arg;
        bool was_app = app_cmd; app_cmd = false;
        if (idx == 0) { st = RX_CMD; return; }
        delay = 2; st = RESP_DELAY;
        if (was_app && idx == 41) {
            acmd41_polls++;
            push_resp48(0x3f, (acmd41_polls >= 2) ? 0xC0FF8000 : 0x00FF8000);
        } else if (idx == 55) { app_cmd = true; push_resp48(55, 0x00000120);
        } else if (idx == 8)  { push_resp48(8, 0x000001AA);
        } else if (idx == 2)  { push_resp136();
        } else if (idx == 9)  { push_csd(csd_csize);
        } else if (idx == 3)  { push_resp48(3, ((uint32_t)rca << 16) | 0x0500);
        } else if (idx == 7)  { push_resp48(7, 0x00000700);
        } else if (idx == 17) { push_resp48(17, 0x00000900); cur_sector = arg;
        } else if (idx == 24) { push_resp48(24, 0x00000900); cur_sector = arg;
        } else                { push_resp48(idx, 0x00000000); }
    }

    // A real card is clocked on the SD-clock RISING edge: it samples host
    // inputs and updates its own outputs there. The host therefore drives on
    // the falling edge (card samples on rising) and samples on the falling edge
    // (card drove on the preceding rising edge -> stable half a period later).
    void on_rising(bool cmd_driven, int cmd_val, bool dat_driven, int dat_val) {
        int cmd_line = cmd_driven ? cmd_val : 1;
        int dat_line = dat_driven ? dat_val : 1;
        if (st == RX_CMD) {
            if (cmd_bits < 0) {
                if (cmd_line == 0) { cmd_sh = 0; cmd_bits = 1; }
            } else {
                cmd_sh = (cmd_sh << 1) | cmd_line;
                if (++cmd_bits == 48) { cmd_bits = -1; decode_command(); }
            }
        } else if (st == WR_RECV) {
            if (rx_left < 0) {
                if (dat_line == 0) { rx_left = 4096 + 16; rx_bitcnt = rx_acc = rx_bytecnt = 0; }
            } else if (rx_left > 0) {
                if (rx_bytecnt < 512) {
                    rx_acc = ((rx_acc << 1) | dat_line) & 0xff;
                    if (++rx_bitcnt == 8) { wbuf[rx_bytecnt++] = rx_acc; rx_bitcnt = rx_acc = 0; }
                }
                if (--rx_left == 0) {
                    store[cur_sector] = wbuf;
                    txbits.clear();
                    int seq[] = {1,1, 0, 0,1,0, 1, 0,0,0,0, 1,1};
                    for (int v : seq) txbits.push_back(v);
                    tx_i = 0; st = WR_DRIVE;
                }
            }
        } else if (st == RESP_DELAY) {
            cmd_out = 1; if (--delay <= 0) st = TX_RESP;
        } else if (st == TX_RESP) {
            cmd_out = resp[resp_i++];
            if (resp_i >= (int)resp.size()) {
                cmd_out = 1;
                if (last_cmd == 17)      begin_read_block(cur_sector);
                else if (last_cmd == 24) begin_write_block(cur_sector);
                else                     st = RX_CMD;
            }
        } else if (st == RD_DRIVE || st == WR_DRIVE) {
            dat_out = txbits[tx_i++];
            if (tx_i >= (int)txbits.size()) { dat_out = 1; st = RX_CMD; }
        }
    }

    void clock_edge(int sd_clk, bool cmd_driven, int cmd_o, bool dat_driven, int dat_o) {
        if (sd_clk && !prev_sd) on_rising(cmd_driven, cmd_o, dat_driven, dat_o);
        prev_sd = sd_clk;
    }
};
