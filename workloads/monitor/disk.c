// The SD card for the monitor: the virtio-blk device (polled, one request at a time), the
// partition table (GPT's EFI System Partition, or an MBR FAT32 partition) and a read-only FAT32
// file system with long names. This is what the autoboot loads /smolrv64/boot.txt and the
// files it names from.
//
// The device's DMA reaches DDR, not the monitor's SRAM, and it is not cache-coherent: the
// queue, the request header and status and the metadata buffers live in DDR scratch at
// DMA_BASE, the CPU's writes there are cleaned to DDR before the device is notified, and a
// buffer the device writes is flushed before the request and invalidated after it. File data
// goes by DMA straight to its destination; only a final partial sector goes through a buffer.
#include "mon.h"

#define VBLK            ((volatile uint32_t *)0x10002000)
#define VM_MAGIC        (0x000 / 4)
#define VM_VERSION      (0x004 / 4)
#define VM_DEVID        (0x008 / 4)
#define VM_DEVFEAT      (0x010 / 4)
#define VM_DEVFEATSEL   (0x014 / 4)
#define VM_DRVFEAT      (0x020 / 4)
#define VM_DRVFEATSEL   (0x024 / 4)
#define VM_QSEL         (0x030 / 4)
#define VM_QNUMMAX      (0x034 / 4)
#define VM_QNUM         (0x038 / 4)
#define VM_QREADY       (0x044 / 4)
#define VM_QNOTIFY      (0x050 / 4)
#define VM_ISR          (0x060 / 4)
#define VM_IACK         (0x064 / 4)
#define VM_STATUS       (0x070 / 4)
#define VM_QDESC        (0x080 / 4)
#define VM_QAVAIL       (0x090 / 4)
#define VM_QUSED        (0x0a0 / 4)
#define VM_CONFIG       (0x100 / 4)
#define ST_ACK 1
#define ST_DRIVER 2
#define ST_DRIVER_OK 4
#define ST_FEATURES_OK 8

// The scratch area (DMA_BASE, mon.h): each piece the CPU and the device share has cache lines of
// its own.
#define QN              8                                       // the queue's size
#define D_NEXT 1
#define D_WRITE 2
struct vq_desc { uint64_t addr; uint32_t len; uint16_t flags; uint16_t next; };
#define VQ_DESC         ((volatile struct vq_desc *)(DMA_BASE + 0x000))
#define VQ_AVAIL        ((volatile uint16_t *)(DMA_BASE + 0x100))   // flags, idx, ring[QN]
#define VQ_USED         ((volatile uint16_t *)(DMA_BASE + 0x200))   // flags, idx, {id, len}[QN]
#define REQ_HDR         ((volatile uint32_t *)(DMA_BASE + 0x300))   // type, 0, sector
#define REQ_ST          ((volatile uint8_t *)(DMA_BASE + 0x340))
#define SECBUF          ((uint8_t *)(DMA_BASE + 0x1000))            // a metadata sector
#define FATBUF          ((uint8_t *)(DMA_BASE + 0x1200))            // the cached FAT sector
#ifndef RD_MAX
#define RD_MAX          256                                      // sectors a request reads
#endif

static void fence(void) { asm volatile ("fence iorw, iorw" ::: "memory"); }

// Zicbom over a range, by 64-byte block: 0 invalidate, 1 clean, 2 flush
void cbo(uint64_t a, uint64_t n, int op)
{
    uint64_t e = a + n;
    for (a &= ~63ul; a < e; a += 64) {
        if (op == 0)      asm volatile ("cbo.inval (%0)" :: "r"(a) : "memory");
        else if (op == 1) asm volatile ("cbo.clean (%0)" :: "r"(a) : "memory");
        else              asm volatile ("cbo.flush (%0)" :: "r"(a) : "memory");
    }
    fence();
}

static int      vq_up;
static uint16_t avail_idx, used_seen;

static void error(const char *what) { puts_("disk: "); puts_(what); putc_('\n'); }

static int vblk_init(void)
{
    if (vq_up) return 0;
    if (VBLK[VM_MAGIC] != 0x74726976u || VBLK[VM_VERSION] != 2 || VBLK[VM_DEVID] != 2) {
        error("no virtio-blk device at 0x10002000");
        return -1;
    }
    VBLK[VM_STATUS] = 0;
    VBLK[VM_STATUS] = ST_ACK;
    VBLK[VM_STATUS] = ST_ACK | ST_DRIVER;
    VBLK[VM_DEVFEATSEL] = 1;
    uint32_t f1 = VBLK[VM_DEVFEAT];
    if (!(f1 & 1)) { error("the device does not offer VIRTIO_F_VERSION_1"); return -1; }
    VBLK[VM_DRVFEATSEL] = 1;  VBLK[VM_DRVFEAT] = f1 & 3;           // VERSION_1, ACCESS_PLATFORM
    VBLK[VM_DRVFEATSEL] = 0;  VBLK[VM_DRVFEAT] = 0;
    VBLK[VM_STATUS] = ST_ACK | ST_DRIVER | ST_FEATURES_OK;
    if (!(VBLK[VM_STATUS] & ST_FEATURES_OK)) { error("the device refused the features"); return -1; }
    VBLK[VM_QSEL] = 0;
    if (VBLK[VM_QNUMMAX] < QN) { error("the device's queue is too small"); return -1; }
    VBLK[VM_QNUM] = QN;
    for (int i = 0; i < 0x380 / 8; i++) ((volatile uint64_t *)DMA_BASE)[i] = 0;
    cbo(DMA_BASE, 0x380, 2);
    VBLK[VM_QDESC]  = (uint32_t)(DMA_BASE + 0x000);  VBLK[VM_QDESC + 1]  = (uint32_t)((DMA_BASE + 0x000) >> 32);
    VBLK[VM_QAVAIL] = (uint32_t)(DMA_BASE + 0x100);  VBLK[VM_QAVAIL + 1] = (uint32_t)((DMA_BASE + 0x100) >> 32);
    VBLK[VM_QUSED]  = (uint32_t)(DMA_BASE + 0x200);  VBLK[VM_QUSED + 1]  = (uint32_t)((DMA_BASE + 0x200) >> 32);
    VBLK[VM_QREADY] = 1;
    VBLK[VM_STATUS] = ST_ACK | ST_DRIVER | ST_FEATURES_OK | ST_DRIVER_OK;
    avail_idx = 0;  used_seen = 0;  vq_up = 1;
    return 0;
}

// The device back to its reset state, so the kernel's driver finds it as the hardware left it.
void disk_quiesce(void)
{
    if (!vq_up) return;
    VBLK[VM_STATUS] = 0;
    VBLK[VM_IACK] = VBLK[VM_ISR];
    vq_up = 0;
}

// Read n sectors at lba into dst (DDR): one request, polled to completion.
static int vblk_read(uint64_t lba, uint64_t dst, uint32_t n)
{
    REQ_HDR[0] = 0;  REQ_HDR[1] = 0;                               // VIRTIO_BLK_T_IN
    REQ_HDR[2] = (uint32_t)lba;  REQ_HDR[3] = (uint32_t)(lba >> 32);
    *REQ_ST = 0xff;
    VQ_DESC[0].addr = DMA_BASE + 0x300;  VQ_DESC[0].len = 16;  VQ_DESC[0].flags = D_NEXT;  VQ_DESC[0].next = 1;
    VQ_DESC[1].addr = dst;  VQ_DESC[1].len = n * 512;  VQ_DESC[1].flags = D_NEXT | D_WRITE;  VQ_DESC[1].next = 2;
    VQ_DESC[2].addr = DMA_BASE + 0x340;  VQ_DESC[2].len = 1;  VQ_DESC[2].flags = D_WRITE;  VQ_DESC[2].next = 0;
    VQ_AVAIL[2 + avail_idx % QN] = 0;
    avail_idx++;
    VQ_AVAIL[1] = avail_idx;
    cbo(dst, (uint64_t)n * 512, 2);
    cbo(DMA_BASE, 0x380, 1);
    VBLK[VM_QNOTIFY] = 0;
    uint64_t t0 = now();
    for (;;) {
        cbo(DMA_BASE + 0x200, 0x80, 0);
        if (VQ_USED[1] == (uint16_t)(used_seen + 1)) break;
        if (now() - t0 > 5 * TIMEBASE_HZ) { error("a read timed out"); disk_quiesce(); return -1; }
    }
    used_seen++;
    VBLK[VM_IACK] = VBLK[VM_ISR];
    cbo(DMA_BASE + 0x340, 1, 0);
    if (*REQ_ST != 0) { error("a read failed"); return -1; }
    cbo(dst, (uint64_t)n * 512, 0);
    return 0;
}

static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t *p) { return rd16(p) | ((uint32_t)rd16(p + 2) << 16); }
static uint64_t rd64(const uint8_t *p) { return rd32(p) | ((uint64_t)rd32(p + 4) << 32); }

// ---- the partition and the file system ----
static int      mounted;
static uint64_t fat_lba, data_lba;
static uint32_t spc, root_clus;
static uint64_t fat_cached = ~0ul;

// the EFI System Partition's type GUID, C12A7328-F81F-11D2-BA4B-00A0C93EC93B, as stored
static const uint8_t esp_guid[16] = { 0x28, 0x73, 0x2a, 0xc1, 0x1f, 0xf8, 0xd2, 0x11,
                                      0xba, 0x4b, 0x00, 0xa0, 0xc9, 0x3e, 0xc9, 0x3b };

static int find_partition(uint64_t *lba)
{
    if (vblk_read(1, (uint64_t)SECBUF, 1)) return -1;
    if (rd64(SECBUF) == 0x5452415020494645ul) {                   // "EFI PART"
        uint64_t ent = rd64(SECBUF + 72);
        uint32_t cnt = rd32(SECBUF + 80), esz = rd32(SECBUF + 84);
        if (esz < 128 || esz > 512 || 512 % esz) { error("odd GPT entry size"); return -1; }
        uint64_t cur = ~0ul;
        for (uint32_t i = 0; i < cnt; i++) {
            uint64_t s = ent + (uint64_t)i * esz / 512;
            if (s != cur && vblk_read(s, (uint64_t)SECBUF, 1)) return -1;
            cur = s;
            const uint8_t *e = SECBUF + (uint64_t)i * esz % 512;
            int j = 0;
            while (j < 16 && e[j] == esp_guid[j]) j++;
            if (j == 16) { *lba = rd64(e + 32); return 0; }
        }
        error("no EFI System Partition in the GPT");
        return -1;
    }
    if (vblk_read(0, (uint64_t)SECBUF, 1)) return -1;
    if (rd16(SECBUF + 510) != 0xaa55) { error("no partition table"); return -1; }
    for (int i = 0; i < 4; i++) {
        const uint8_t *e = SECBUF + 446 + 16 * i;
        if (e[4] == 0xef || e[4] == 0x0b || e[4] == 0x0c) { *lba = rd32(e + 8); return 0; }
    }
    error("no FAT32 or EFI partition in the MBR");
    return -1;
}

static int mount(void)
{
    uint64_t part;
    if (mounted) return 0;
    if (vblk_init() || find_partition(&part)) return -1;
    if (vblk_read(part, (uint64_t)SECBUF, 1)) return -1;
    if (rd16(SECBUF + 510) != 0xaa55 || rd16(SECBUF + 11) != 512 || rd16(SECBUF + 22) != 0
        || !SECBUF[13] || !SECBUF[16]) {
        error("the partition is not FAT32 with 512-byte sectors");
        return -1;
    }
    spc       = SECBUF[13];
    fat_lba   = part + rd16(SECBUF + 14);
    data_lba  = fat_lba + (uint64_t)SECBUF[16] * rd32(SECBUF + 36);
    root_clus = rd32(SECBUF + 44);
    fat_cached = ~0ul;
    mounted = 1;
    return 0;
}

#define FAT_END(c) ((c) < 2 || (c) >= 0x0ffffff8u)

static uint32_t fat_next(uint32_t c)
{
    uint64_t s = fat_lba + c / 128;
    if (s != fat_cached) {
        if (vblk_read(s, (uint64_t)FATBUF, 1)) return 0x0fffffffu;
        fat_cached = s;
    }
    return rd32(FATBUF + (c % 128) * 4) & 0x0fffffffu;
}

static uint64_t clus_lba(uint32_t c) { return data_lba + (uint64_t)(c - 2) * spc; }

struct dent { char name[256]; uint32_t clus, size; uint8_t attr; };

static char lower(char c) { return (c >= 'A' && c <= 'Z') ? c + 32 : c; }

// each directory entry of the directory at cluster c, long name assembled; fn returns 1 to stop
static int dir_walk(uint32_t c, int (*fn)(struct dent *, void *), void *ctx)
{
    static const uint8_t lfn_at[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
    static struct dent e;
    static char lfn[256];
    int have_lfn = 0;
    while (!FAT_END(c)) {
        for (uint32_t s = 0; s < spc; s++) {
            if (vblk_read(clus_lba(c) + s, (uint64_t)SECBUF, 1)) return -1;
            for (int off = 0; off < 512; off += 32) {
                const uint8_t *d = SECBUF + off;
                if (d[0] == 0) return 0;
                if (d[0] == 0xe5) { have_lfn = 0; continue; }
                if (d[11] == 0x0f) {
                    int seq = d[0] & 0x1f;
                    if (d[0] & 0x40) { for (int i = 0; i < 256; i++) lfn[i] = 0; have_lfn = 1; }
                    for (int i = 0; i < 13 && seq; i++) {
                        int pos = (seq - 1) * 13 + i;
                        uint16_t u = rd16(d + lfn_at[i]);
                        if (pos < 255 && u != 0xffff) lfn[pos] = u < 0x80 ? (char)u : '?';
                    }
                    continue;
                }
                if (d[11] & 0x08) { have_lfn = 0; continue; }       // the volume label
                int n = 0;
                if (have_lfn) {
                    while (n < 255 && lfn[n]) { e.name[n] = lfn[n]; n++; }
                } else {
                    for (int i = 0; i < 8 && d[i] != ' '; i++) e.name[n++] = (i == 0 && d[0] == 0x05) ? (char)0xe5 : d[i];
                    if (d[8] != ' ') {
                        e.name[n++] = '.';
                        for (int i = 8; i < 11 && d[i] != ' '; i++) e.name[n++] = d[i];
                    }
                }
                e.name[n] = 0;
                e.clus = ((uint32_t)rd16(d + 20) << 16) | rd16(d + 26);
                e.size = rd32(d + 28);
                e.attr = d[11];
                have_lfn = 0;
                if (fn(&e, ctx)) return 1;
            }
        }
        c = fat_next(c);
    }
    return 0;
}

struct want { const char *name; int len; struct dent *out; };

static int match(struct dent *e, void *ctx)
{
    struct want *w = ctx;
    int i = 0;
    while (i < w->len && e->name[i] && lower(e->name[i]) == lower(w->name[i])) i++;
    if (i != w->len || e->name[i]) return 0;
    for (i = 0; (w->out->name[i] = e->name[i]); i++) ;
    w->out->clus = e->clus;  w->out->size = e->size;  w->out->attr = e->attr;
    return 1;
}

// the entry a path names; "" and "/" name the root directory. -2: no such file, -1: the card or
// its file system failed
static int find(const char *path, struct dent *out)
{
    if (mount()) return -1;
    out->clus = root_clus;  out->attr = 0x10;  out->size = 0;  out->name[0] = 0;
    for (;;) {
        while (*path == '/') path++;
        if (!*path) return 0;
        if (!(out->attr & 0x10)) { error("not a directory"); return -1; }
        struct want w = { path, 0, out };
        while (path[w.len] && path[w.len] != '/') w.len++;
        uint32_t c = out->clus ? out->clus : root_clus;
        int r = dir_walk(c, match, &w);
        if (r < 0) return -1;
        if (r == 0) { puts_("disk: no such file: "); puts_(path); putc_('\n'); return -2; }
        path += w.len;
    }
}

static int show(struct dent *e, void *ctx)
{
    (void)ctx;
    if (e->name[0] == '.') return 0;
    puts_("  ");
    puthex32(e->size);
    puts_("  ");
    puts_(e->name);
    if (e->attr & 0x10) putc_('/');
    putc_('\n');
    return 0;
}

int disk_list(const char *path)
{
    struct dent d;
    if (find(path, &d)) return -1;
    if (!(d.attr & 0x10)) { show(&d, 0); return 0; }
    return dir_walk(d.clus ? d.clus : root_clus, show, 0) < 0 ? -1 : 0;
}

// Load a file to addr; refuse one larger than max. *size is its length. -2: no such file.
int disk_load(const char *path, uint64_t addr, uint64_t max, uint64_t *size)
{
    struct dent d;
    int r = find(path, &d);
    if (r) return r;
    if (d.attr & 0x10) { error("a directory, not a file"); return -1; }
    if (d.size > max) { error("the file is too large"); return -1; }
    uint64_t rem = d.size, dst = addr, t0 = now(), mark = 1ul << 20;
    uint32_t c = d.clus;
    while (rem) {
        if (FAT_END(c)) { error("the cluster chain ends before the file does"); return -1; }
        uint32_t last = c, n = 1;                   // a run of consecutive clusters
        while ((uint64_t)n * spc * 512 < rem) {
            uint32_t nx = fat_next(last);
            if (nx != last + 1) break;
            last = nx;  n++;
        }
        uint64_t take = (uint64_t)n * spc * 512;
        if (take > rem) take = rem;
        uint64_t lba = clus_lba(c), full = take / 512;
        for (uint64_t s = 0; s < full; ) {
            uint32_t k = full - s > RD_MAX ? RD_MAX : (uint32_t)(full - s);
            if (vblk_read(lba + s, dst + s * 512, k)) return -1;
            s += k;
        }
        if (take % 512) {                           // the final partial sector, through SECBUF
            if (vblk_read(lba + full, (uint64_t)SECBUF, 1)) return -1;
            for (uint64_t i = 0; i < take % 512; i++) ((volatile uint8_t *)dst)[full * 512 + i] = SECBUF[i];
        }
        dst += take;  rem -= take;
        if (dst - addr >= mark) { putc_('.'); mark += 1ul << 20; }
        if (rem) c = fat_next(last);
    }
    uint64_t ms = (now() - t0) * 1000 / TIMEBASE_HZ;
    if (d.size >= (1ul << 20)) putc_('\n');
    puts_(path);
    puts_(": ");
    putdec(d.size);
    puts_(" bytes in ");
    putdec(ms);
    puts_(" ms\n");
    *size = d.size;
    return 0;
}
