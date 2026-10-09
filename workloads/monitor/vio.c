// virtio-mmio for the monitor's polled drivers (disk.c, net.c): device bring-up and split
// virtqueues in DDR scratch.
//
// The devices' DMA reaches DDR, not the monitor's SRAM, and it is not cache-coherent: what the
// CPU writes for a device is cleaned to DDR before the device is notified, and what a device
// writes is invalidated before the CPU reads it. A queue takes the depth the device reports
// (QueueNumMax): the devices index their rings modulo that depth, whatever QueueNum says.
#include "mon.h"

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
#define ST_ACK          1
#define ST_DRIVER       2
#define ST_DRIVER_OK    4
#define ST_FEATURES_OK  8

// A queue's 16 KiB at its base (VQ_SPAN): the descriptors, then the available ring at +0x1000
// (flags, idx, ring[n]), then the used ring at +0x2000 (flags, idx, {id, len}[n]).
#define AVAIL(q)        ((volatile uint16_t *)((q)->base + 0x1000))
#define USED(q)         ((volatile uint16_t *)((q)->base + 0x2000))

static void fence(void) { asm volatile ("fence iorw, iorw" ::: "memory"); }

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

static int fail(const char *name, const char *what)
{
    puts_(name); puts_(": "); puts_(what); putc_('\n');
    return -1;
}

int vio_start(volatile uint32_t *d, uint32_t id, const char *name)
{
    if (d[VM_MAGIC] != 0x74726976u || d[VM_VERSION] != 2 || d[VM_DEVID] != id) {
        puts_(name); puts_(": no virtio device of that kind: magic "); puthex32(d[VM_MAGIC]);
        puts_(" version "); putdec(d[VM_VERSION]); puts_(" id "); putdec(d[VM_DEVID]);
        puts_(", want id "); putdec(id); putc_('\n');
        return -1;
    }
    d[VM_STATUS] = 0;
    d[VM_STATUS] = ST_ACK;
    d[VM_STATUS] = ST_ACK | ST_DRIVER;
    d[VM_DEVFEATSEL] = 1;
    uint32_t f1 = d[VM_DEVFEAT];
    if (!(f1 & 1)) return fail(name, "the device does not offer VIRTIO_F_VERSION_1");
    d[VM_DRVFEATSEL] = 1;  d[VM_DRVFEAT] = f1 & 3;                // VERSION_1, ACCESS_PLATFORM
    d[VM_DRVFEATSEL] = 0;  d[VM_DRVFEAT] = 0;
    d[VM_STATUS] = ST_ACK | ST_DRIVER | ST_FEATURES_OK;
    if (!(d[VM_STATUS] & ST_FEATURES_OK)) return fail(name, "the device refused the features");
    return 0;
}

int vio_queue(struct vq *q, volatile uint32_t *d, uint32_t sel, uint64_t base, const char *name)
{
    d[VM_QSEL] = sel;
    uint32_t n = d[VM_QNUMMAX];
    if (n == 0 || n > VQ_MAX || (n & (n - 1))) {
        puts_(name); puts_(": queue "); putdec(sel); puts_("'s depth "); putdec(n);
        puts_(" is not a power of two in 1..256\n");
        return -1;
    }
    d[VM_QNUM] = n;
    for (int i = 0; i < VQ_SPAN / 8; i++) ((volatile uint64_t *)base)[i] = 0;
    cbo(base, VQ_SPAN, 2);
    d[VM_QDESC]  = (uint32_t)base;             d[VM_QDESC + 1]  = (uint32_t)(base >> 32);
    d[VM_QAVAIL] = (uint32_t)(base + 0x1000);  d[VM_QAVAIL + 1] = (uint32_t)((base + 0x1000) >> 32);
    d[VM_QUSED]  = (uint32_t)(base + 0x2000);  d[VM_QUSED + 1]  = (uint32_t)((base + 0x2000) >> 32);
    d[VM_QREADY] = 1;
    q->dev = d;  q->sel = sel;  q->base = base;  q->n = (uint16_t)n;  q->avail = 0;  q->used = 0;
    return 0;
}

void vio_ready(volatile uint32_t *d)
{
    d[VM_STATUS] = ST_ACK | ST_DRIVER | ST_FEATURES_OK | ST_DRIVER_OK;
}

// The device back to its reset state, so the kernel's driver finds it as the hardware left it.
void vio_reset(volatile uint32_t *d)
{
    d[VM_STATUS] = 0;
    d[VM_IACK] = d[VM_ISR];
}

void vq_add(struct vq *q, uint16_t head)
{
    volatile uint16_t *e = &AVAIL(q)[2 + q->avail % q->n];
    *e = head;
    cbo((uint64_t)e, 2, 1);
    q->avail++;
}

void vq_kick(struct vq *q)
{
    AVAIL(q)[1] = q->avail;
    cbo((uint64_t)AVAIL(q), 4, 1);
    q->dev[VM_QNOTIFY] = q->sel;
}

int vq_used(struct vq *q, uint32_t *len)
{
    cbo((uint64_t)USED(q), 4, 0);
    if (USED(q)[1] == q->used) return -1;
    volatile uint32_t *e = (volatile uint32_t *)(USED(q) + 2) + 2 * (q->used % q->n);
    cbo((uint64_t)e, 8, 0);
    int id = (int)e[0];
    if (len) *len = e[1];
    q->used++;
    q->dev[VM_IACK] = q->dev[VM_ISR];
    return id;
}
