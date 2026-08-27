// aesbench -- deterministic T-table AES-128, the kernel GB5's AES-XTS actually runs.
//
// WHY THIS EXISTS. GB5 scores AES-XTS 1 (950.6 KB/sec = 175 cycles/byte at 166.67 MHz)
// and it is the whole Crypto score, but GB5 cannot run one workload: isolating it means
// killing a multi-hour run partway, and two such captures differed 3.6% in instruction
// count purely from where the kill landed. That is useless for judging a 5% change, and
// a window that also contains process startup over NFS is worse than useless -- it once
// produced a "+19.8% on AES-XTS" that was mostly startup.
//
// This runs the same shape deterministically in seconds: 10 serial rounds per block,
// 16 independent table lookups per round. Loads and XORs, nothing else -- exactly the
// pattern that should reward a deeper scheduler and multiple outstanding loads, and the
// reason it is the right yardstick for choosing a scheduler size.
//
// Self-checking against the FIPS-197 vector, because a benchmark that is not also a test
// will happily measure the wrong computation.
//
// Bare-metal M-mode, boots at 0x8000_0000:  FW=aesbench.bin ooo2/run-ooo2-linux.sh

typedef unsigned char  uint8_t;
typedef unsigned int   uint32_t;
typedef unsigned long  uint64_t;

#define UART     ((volatile uint8_t *)0x10000000)
#define LSR_THRE 0x20

static void putc_(uint8_t c) { while (!(UART[5] & LSR_THRE)); UART[0] = c; }
static void puts_(const char *s) { while (*s) { if (*s == '\n') putc_('\r'); putc_(*s++); } }
static void puthex(uint64_t v, int nyb) {
    for (int i = (nyb - 1) * 4; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]);
}
static void putdec(uint64_t v) {
    char b[24]; int n = 0;
    if (!v) { putc_('0'); return; }
    while (v) { b[n++] = '0' + (v % 10); v /= 10; }
    while (n) putc_(b[--n]);
}

static const uint8_t S[256] = {
0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16 };

// 4 KiB of tables -- the working set the D$ must hold, and the reason this workload is
// hit-bound (GB5 measures 0.28% D$ miss on it) rather than miss-bound.
static uint32_t Te0[256], Te1[256], Te2[256], Te3[256];
static uint32_t rk[44];

static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
static uint8_t  xt(uint8_t a) { return (uint8_t)((a << 1) ^ ((a >> 7) * 0x1b)); }

static void tables(void) {
    for (int i = 0; i < 256; i++) {
        uint8_t s = S[i], s2 = xt(s), s3 = s2 ^ s;
        Te0[i] = ((uint32_t)s2 << 24) | ((uint32_t)s << 16) | ((uint32_t)s << 8) | s3;
        Te1[i] = rotr(Te0[i], 8); Te2[i] = rotr(Te0[i], 16); Te3[i] = rotr(Te0[i], 24);
    }
}

static void expand(const uint8_t *key) {
    for (int i = 0; i < 4; i++)
        rk[i] = ((uint32_t)key[4*i] << 24) | ((uint32_t)key[4*i+1] << 16)
              | ((uint32_t)key[4*i+2] << 8) | key[4*i+3];
    uint8_t rc = 1;
    for (int i = 4; i < 44; i++) {
        uint32_t t = rk[i-1];
        if (i % 4 == 0) {
            t = ((uint32_t)S[(t >> 16) & 0xff] << 24) | ((uint32_t)S[(t >> 8) & 0xff] << 16)
              | ((uint32_t)S[t & 0xff] << 8)          |  S[(t >> 24) & 0xff];
            t ^= (uint32_t)rc << 24; rc = xt(rc);
        }
        rk[i] = rk[i-4] ^ t;
    }
}

// The measured kernel. 10 rounds, serially dependent; 16 independent loads inside each.
static void encrypt(const uint8_t *in, uint8_t *out) {
    uint32_t s0, s1, s2, s3, t0, t1, t2, t3;
    s0 = (((uint32_t)in[0]<<24)|((uint32_t)in[1]<<16)|((uint32_t)in[2]<<8)|in[3])  ^ rk[0];
    s1 = (((uint32_t)in[4]<<24)|((uint32_t)in[5]<<16)|((uint32_t)in[6]<<8)|in[7])  ^ rk[1];
    s2 = (((uint32_t)in[8]<<24)|((uint32_t)in[9]<<16)|((uint32_t)in[10]<<8)|in[11])^ rk[2];
    s3 = (((uint32_t)in[12]<<24)|((uint32_t)in[13]<<16)|((uint32_t)in[14]<<8)|in[15])^rk[3];
    for (int r = 1; r < 10; r++) {
        const uint32_t *k = &rk[4*r];
        t0 = Te0[s0>>24] ^ Te1[(s1>>16)&0xff] ^ Te2[(s2>>8)&0xff] ^ Te3[s3&0xff] ^ k[0];
        t1 = Te0[s1>>24] ^ Te1[(s2>>16)&0xff] ^ Te2[(s3>>8)&0xff] ^ Te3[s0&0xff] ^ k[1];
        t2 = Te0[s2>>24] ^ Te1[(s3>>16)&0xff] ^ Te2[(s0>>8)&0xff] ^ Te3[s1&0xff] ^ k[2];
        t3 = Te0[s3>>24] ^ Te1[(s0>>16)&0xff] ^ Te2[(s1>>8)&0xff] ^ Te3[s2&0xff] ^ k[3];
        s0 = t0; s1 = t1; s2 = t2; s3 = t3;
    }
    const uint32_t *k = &rk[40];
    t0 = ((uint32_t)S[s0>>24]<<24)|((uint32_t)S[(s1>>16)&0xff]<<16)|((uint32_t)S[(s2>>8)&0xff]<<8)|S[s3&0xff];
    t1 = ((uint32_t)S[s1>>24]<<24)|((uint32_t)S[(s2>>16)&0xff]<<16)|((uint32_t)S[(s3>>8)&0xff]<<8)|S[s0&0xff];
    t2 = ((uint32_t)S[s2>>24]<<24)|((uint32_t)S[(s3>>16)&0xff]<<16)|((uint32_t)S[(s0>>8)&0xff]<<8)|S[s1&0xff];
    t3 = ((uint32_t)S[s3>>24]<<24)|((uint32_t)S[(s0>>16)&0xff]<<16)|((uint32_t)S[(s1>>8)&0xff]<<8)|S[s2&0xff];
    t0 ^= k[0]; t1 ^= k[1]; t2 ^= k[2]; t3 ^= k[3];
    out[0]=t0>>24; out[1]=t0>>16; out[2]=t0>>8; out[3]=t0;
    out[4]=t1>>24; out[5]=t1>>16; out[6]=t1>>8; out[7]=t1;
    out[8]=t2>>24; out[9]=t2>>16; out[10]=t2>>8; out[11]=t2;
    out[12]=t3>>24;out[13]=t3>>16;out[14]=t3>>8; out[15]=t3;
}

static uint64_t rdcycle(void)  { uint64_t v; __asm__ volatile("rdcycle  %0":"=r"(v)); return v; }
static uint64_t rdinstr(void)  { uint64_t v; __asm__ volatile("rdinstret %0":"=r"(v)); return v; }

// A CPI stack for the AES kernel ALONE. The GB5 numbers for this workload were taken over
// a window that also held process startup, so they cannot say where AES itself spends its
// cycles. These can. Event numbers are src/csr_file.v's HPMEV_*.
#define EV_FE_IC  0x0312   /* fetch window empty  */
#define EV_FE_ALN 0x0313   /* bytes, but no COMPLETE instruction -- the RVC aligner */
#define EV_FE_QUE 0x0314   /* had one; F/X queue empty */
#define EV_FE_BUB 0x0310   /* total: X idle, frontend supplied nothing */
#define EV_ICMISS 0x0112
#define EV_ST_MEM 0x0300
#define SETEV(n, e) __asm__ volatile("csrw 0x32" #n ", %0" :: "r"((uint64_t)(e)))
#define RDCNT(n)    ({ uint64_t v; __asm__ volatile("csrr %0, 0xB0" #n : "=r"(v)); v; })

static void hpm_setup(void) {
    SETEV(3, EV_FE_IC);  SETEV(4, EV_FE_ALN); SETEV(5, EV_FE_QUE);
    SETEV(6, EV_FE_BUB); SETEV(7, EV_ICMISS);  SETEV(8, EV_ST_MEM);
}
static void pct(const char *nm, uint64_t v, uint64_t tot) {
    puts_(nm); putdec(v);
    puts_(" ("); putdec(v * 100 / tot); putc_('.'); putdec((v * 1000 / tot) % 10); puts_("%)\n");
}

// FOOTPRINT MATTERS AS MUCH AS THE BYTE COUNT. An earlier version encrypted an 8 KiB
// buffer eight times: after pass 1 everything was resident in the 64 KiB D$, D$ misses
// were literally 0, and the kernel looked purely frontend-bound. GB5's AES-XTS streams
// through a buffer instead and misses ~0.28% -- low, but real traffic on every line.
// Same total bytes, one pass, 128 KiB of footprint = 2x the D$, so lines are actually
// fetched. Without this the benchmark disagrees with GB5 about where the cycles go and
// would mis-rank the work.
#define NBLK 4096                /* 64 KiB in + 64 KiB out, streamed once */
static uint8_t buf[NBLK * 16], obuf[NBLK * 16];

int main(void) {
    static const uint8_t key[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    static const uint8_t pt[16]  = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
                                    0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    static const uint8_t ct[16]  = {0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
                                    0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a};
    uint8_t chk[16];
    tables(); expand(key);

    // A benchmark that is not also a test measures the wrong computation happily.
    encrypt(pt, chk);
    for (int i = 0; i < 16; i++)
        if (chk[i] != ct[i]) { puts_("aesbench: FIPS-197 MISMATCH\n"); for(;;); }
    puts_("aesbench: FIPS-197 vector ok\n");

    for (int i = 0; i < NBLK * 16; i++) buf[i] = (uint8_t)(i * 7 + 13);
    // Deliberately NOT warmed: streaming past a 64 KiB cache is the point.

    hpm_setup();
    uint64_t e3 = RDCNT(3), e4 = RDCNT(4), e5 = RDCNT(5);
    uint64_t e6 = RDCNT(6), e7 = RDCNT(7), e8 = RDCNT(8);
    uint64_t c0 = rdcycle(), i0 = rdinstr();
    for (int i = 0; i < NBLK; i++) encrypt(buf + 16*i, obuf + 16*i);
    uint64_t c = rdcycle() - c0, n = rdinstr() - i0;
    e3 = RDCNT(3) - e3; e4 = RDCNT(4) - e4; e5 = RDCNT(5) - e5;
    e6 = RDCNT(6) - e6; e7 = RDCNT(7) - e7; e8 = RDCNT(8) - e8;

    const uint64_t bytes = (uint64_t)NBLK * 16;
    puts_("aesbench: bytes=");   putdec(bytes);
    puts_(" cycles=");           putdec(c);
    puts_(" insn=");             putdec(n);
    puts_("\naesbench: cyc/byte=");
    putdec(c / bytes); putc_('.'); putdec((c * 100 / bytes) % 100);
    puts_("  insn/byte=");
    putdec(n / bytes); putc_('.'); putdec((n * 100 / bytes) % 100);
    puts_("  IPC=0.");
    putdec(n * 1000 / c);
    puts_("\n");
    pct("aesbench: FE_BUB total ", e6, c);
    pct("aesbench:   FE_IC  (I$)  ", e3, c);
    pct("aesbench:   FE_ALN (rvc) ", e4, c);
    pct("aesbench:   FE_QUE (F/X) ", e5, c);
    pct("aesbench: ST_MEM        ", e8, c);
    puts_("aesbench: I$miss="); putdec(e7);
    puts_("\naesbench: done\n");
    for (;;);
}
