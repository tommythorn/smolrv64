// The network for the monitor: the virtio-net device (polled), Ethernet, ARP, IPv4 and UDP, a
// DHCP client for the board's address and a TFTP client that loads files from the server on
// coffee (TFTP_SERVER, tftpd-hpa on port 69) into memory. This is the development boot path;
// the SD card (disk.c) is for demos.
//
// The device takes one descriptor per frame, a 12-byte virtio_net_hdr_v1 and then the Ethernet
// frame, and it does not filter by address, so the board's MAC is the locally administered one
// below. TFTP asks for 1468-byte blocks (RFC 2348): a block fills a 1500-byte IP packet.
#include "mon.h"

#define VNET            ((volatile uint32_t *)0x10003000)
#ifndef TFTP_SERVER
#define TFTP_SERVER     0xC0A80191u                     // 192.168.1.145, coffee
#endif
#define NET_BASE        (DMA_BASE + 0x20000)
#define RXQ_BASE        NET_BASE
#define TXQ_BASE        (NET_BASE + 0x1000)
#define TXBUF           (NET_BASE + 0x2000)
#define RXBUF           (NET_BASE + 0x4000)             // VQ_MAX buffers of BUFSZ
#define BUFSZ           2048
#define HDR             12                              // virtio_net_hdr_v1
#define BLKSIZE         1468
#define SEC             TIMEBASE_HZ

static struct vq rxq, txq;
static int       net_up;
static const uint8_t my_mac[6] = { 0x02, 'S', 'M', 'O', 'L', 0x01 };
static const uint8_t bcast[6]  = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static uint32_t my_ip, netmask, gateway;
static uint32_t arp_ip;                                  // a cache of one: the next hop
static uint8_t  arp_mac[6];

static void error(const char *what) { puts_("net: "); puts_(what); putc_('\n'); }

static uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static uint32_t be32(const uint8_t *p) { return (uint32_t)be16(p) << 16 | be16(p + 2); }
static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void put32(uint8_t *p, uint32_t v) { put16(p, v >> 16); put16(p + 2, v); }
static void copy(void *d, const void *s, uint64_t n)
{
    uint8_t *dd = d;  const uint8_t *ss = s;
    while (n--) *dd++ = *ss++;
}
static int same(const uint8_t *a, const uint8_t *b, int n)
{
    while (n--) if (*a++ != *b++) return 0;
    return 1;
}
static void putip(uint32_t a)
{
    for (int i = 24; i >= 0; i -= 8) { putdec((a >> i) & 0xff); if (i) putc_('.'); }
}

// ---- the device ----
static int net_init(void)
{
    if (net_up) return 0;
    if (vio_start(VNET, 1, "net") || vio_queue(&rxq, VNET, 0, RXQ_BASE, "net") ||
        vio_queue(&txq, VNET, 1, TXQ_BASE, "net")) return -1;
    for (uint16_t i = 0; i < rxq.n; i++) {
        volatile struct vq_desc *d = &VQ_DESC(&rxq)[i];
        d->addr = RXBUF + (uint64_t)i * BUFSZ;  d->len = BUFSZ;  d->flags = D_WRITE;  d->next = 0;
        vq_add(&rxq, i);
    }
    cbo(RXBUF, (uint64_t)rxq.n * BUFSZ, 2);
    vio_ready(VNET);
    vq_kick(&rxq);
    net_up = 1;
    return 0;
}

void net_quiesce(void)
{
    if (!net_up) return;
    vio_reset(VNET);
    net_up = 0;  my_ip = 0;  arp_ip = 0;
}

static uint8_t *tx_frame(void) { return (uint8_t *)(TXBUF + HDR); }

static int eth_send(const uint8_t *dst, uint16_t type, uint32_t len)   // len: past the Ethernet header
{
    uint8_t *f = tx_frame();
    copy(f, dst, 6);  copy(f + 6, my_mac, 6);  put16(f + 12, type);
    len += 14;
    while (len < 60) f[len++] = 0;
    for (int i = 0; i < HDR; i++) ((uint8_t *)TXBUF)[i] = 0;
    volatile struct vq_desc *d = VQ_DESC(&txq);
    d->addr = TXBUF;  d->len = HDR + len;  d->flags = 0;  d->next = 0;
    vq_add(&txq, 0);
    cbo(TXBUF, HDR + len, 1);
    vq_kick(&txq);
    uint64_t t0 = now();
    while (vq_used(&txq, 0) < 0)
        if (now() - t0 > SEC) { error("the device did not send a frame"); return -1; }
    return 0;
}

// A received frame, held until rx_done; at most one at a time.
static int rx_id = -1;
static uint8_t *rx_frame(uint32_t *len)
{
    uint32_t n;
    int id = vq_used(&rxq, &n);
    if (id < 0) return 0;
    if ((uint32_t)id >= rxq.n || n < HDR + 14 || n > BUFSZ) { error("a malformed receive"); return 0; }
    uint64_t b = RXBUF + (uint64_t)id * BUFSZ;
    cbo(b, n, 0);
    rx_id = id;
    *len = n - HDR;
    return (uint8_t *)(b + HDR);
}
static void rx_done(void)
{
    uint64_t b = RXBUF + (uint64_t)rx_id * BUFSZ;
    cbo(b, BUFSZ, 0);
    vq_add(&rxq, (uint16_t)rx_id);
    vq_kick(&rxq);
    rx_id = -1;
}

// ---- ARP, IPv4, UDP ----
static void arp_send(uint16_t op, const uint8_t *tha, uint32_t tpa)
{
    uint8_t *a = tx_frame() + 14;
    put16(a, 1);  put16(a + 2, 0x0800);  a[4] = 6;  a[5] = 4;  put16(a + 6, op);
    copy(a + 8, my_mac, 6);  put32(a + 14, my_ip);
    copy(a + 18, op == 1 ? (const uint8_t *)"\0\0\0\0\0\0" : tha, 6);  put32(a + 24, tpa);
    eth_send(op == 1 ? bcast : tha, 0x0806, 28);
}

static uint16_t ip_sum(const uint8_t *p, int n)
{
    uint32_t s = 0;
    for (int i = 0; i < n; i += 2) s += be16(p + i);
    while (s >> 16) s = (s & 0xffff) + (s >> 16);
    return (uint16_t)~s;
}

// The UDP datagram most recently received for the polled port.
static uint8_t  udp_buf[1500];
static uint32_t udp_len, udp_src;
static uint16_t udp_sport;

// Receive one frame: ARP is handled here; a UDP datagram for dport is copied to udp_buf.
// Returns 1 for such a datagram, 0 otherwise.
static int poll_frame(uint16_t dport)
{
    uint32_t n;
    uint8_t *f = rx_frame(&n);
    if (!f) return 0;
    int got = 0;
    uint16_t type = be16(f + 12);
    uint8_t *p = f + 14;
    if (type == 0x0806 && n >= 14 + 28 && be16(p + 6) == 1 && my_ip && be32(p + 24) == my_ip) {
        uint8_t sha[6];  copy(sha, p + 8, 6);
        uint32_t spa = be32(p + 14);
        rx_done();
        arp_send(2, sha, spa);
        return 0;
    }
    if (type == 0x0806 && n >= 14 + 28 && be16(p + 6) == 2 && be32(p + 14) == arp_ip)
        copy(arp_mac, p + 8, 6);
    if (type == 0x0800 && n >= 14 + 28 && (p[0] >> 4) == 4 && p[9] == 17) {
        uint32_t ihl = (p[0] & 15) * 4, tot = be16(p + 2);
        uint8_t *u = p + ihl;
        uint32_t dst = be32(p + 16);
        if (tot <= n - 14 && ihl >= 20 && be16(u + 2) == dport &&
            (dst == my_ip || dst == 0xffffffffu || !my_ip)) {
            udp_len = be16(u + 4) - 8;
            if (udp_len > sizeof udp_buf || udp_len > tot - ihl - 8) udp_len = 0;
            else {
                copy(udp_buf, u + 8, udp_len);
                udp_src = be32(p + 12);  udp_sport = be16(u);
                got = 1;
            }
        }
    }
    rx_done();
    return got;
}

static int udp_send(uint32_t dst, uint16_t sport, uint16_t dport, const uint8_t *data, uint32_t len)
{
    const uint8_t *mac = bcast;
    if (dst != 0xffffffffu) {
        uint32_t hop = ((dst ^ my_ip) & netmask) && gateway ? gateway : dst;
        if (arp_ip != hop) {
            arp_ip = hop;  copy(arp_mac, bcast, 6);
            for (int t = 0; t < 4 && same(arp_mac, bcast, 6); t++) {
                arp_send(1, 0, hop);
                for (uint64_t t0 = now(); now() - t0 < SEC / 2 && same(arp_mac, bcast, 6); ) poll_frame(0);
            }
            if (same(arp_mac, bcast, 6)) { arp_ip = 0; puts_("net: no ARP reply from "); putip(hop); putc_('\n'); return -1; }
        }
        mac = arp_mac;
    }
    uint8_t *ip = tx_frame() + 14, *u = ip + 20;
    ip[0] = 0x45;  ip[1] = 0;  put16(ip + 2, 28 + len);  put16(ip + 4, 0);  put16(ip + 6, 0x4000);
    ip[8] = 64;  ip[9] = 17;  put16(ip + 10, 0);  put32(ip + 12, my_ip);  put32(ip + 16, dst);
    put16(ip + 10, ip_sum(ip, 20));
    put16(u, sport);  put16(u + 2, dport);  put16(u + 4, 8 + len);  put16(u + 6, 0);
    copy(u + 8, data, len);
    return eth_send(mac, 0x0800, 28 + len);
}

// Wait up to tmo for a datagram on port; 1 if one arrived.
static int udp_wait(uint16_t port, uint64_t tmo)
{
    for (uint64_t t0 = now(); now() - t0 < tmo; )
        if (poll_frame(port)) return 1;
    return 0;
}

// ---- DHCP ----
static uint8_t pkt[600];

static uint32_t dhcp_msg(uint8_t type, uint32_t xid, uint32_t req_ip, uint32_t server)
{
    for (unsigned i = 0; i < sizeof pkt; i++) pkt[i] = 0;
    pkt[0] = 1;  pkt[1] = 1;  pkt[2] = 6;  put32(pkt + 4, xid);  put16(pkt + 10, 0x8000);
    copy(pkt + 28, my_mac, 6);
    put32(pkt + 236, 0x63825363u);
    uint8_t *o = pkt + 240;
    *o++ = 53;  *o++ = 1;  *o++ = type;
    if (req_ip) { *o++ = 50;  *o++ = 4;  put32(o, req_ip);  o += 4; }
    if (server) { *o++ = 54;  *o++ = 4;  put32(o, server);  o += 4; }
    *o++ = 55;  *o++ = 2;  *o++ = 1;  *o++ = 3;
    *o++ = 255;
    return (uint32_t)(o - pkt);
}

// The options of the reply in udp_buf: its type, and the fields we use.
static int dhcp_reply(uint32_t xid, uint32_t *server)
{
    if (udp_len < 240 || udp_buf[0] != 2 || be32(udp_buf + 4) != xid || be32(udp_buf + 236) != 0x63825363u) return 0;
    int type = 0;
    for (uint32_t i = 240; i + 1 < udp_len && udp_buf[i] != 255; ) {
        uint8_t c = udp_buf[i], l = udp_buf[i + 1];
        const uint8_t *v = udp_buf + i + 2;
        if (c == 0) { i++; continue; }
        if (i + 2 + l > udp_len) break;
        if (c == 53 && l >= 1) type = v[0];
        if (c == 1 && l >= 4) netmask = be32(v);
        if (c == 3 && l >= 4) gateway = be32(v);
        if (c == 54 && l >= 4) *server = be32(v);
        i += 2 + l;
    }
    return type;
}

static int dhcp(void)
{
    uint32_t xid = (uint32_t)now() ^ 0x534d4f4cu;
    for (int attempt = 0; attempt < 4; attempt++) {
        uint32_t server = 0, offer = 0;
        my_ip = 0;  netmask = 0;  gateway = 0;
        if (udp_send(0xffffffffu, 68, 67, pkt, dhcp_msg(1, xid, 0, 0))) return -1;
        for (uint64_t t0 = now(); now() - t0 < 2 * SEC && !offer; )
            if (udp_wait(68, SEC / 4) && dhcp_reply(xid, &server) == 2) offer = be32(udp_buf + 16);
        if (!offer) continue;
        if (udp_send(0xffffffffu, 68, 67, pkt, dhcp_msg(3, xid, offer, server))) return -1;
        for (uint64_t t0 = now(); now() - t0 < 2 * SEC; )
            if (udp_wait(68, SEC / 4)) {
                int t = dhcp_reply(xid, &server);
                if (t == 5) {
                    my_ip = be32(udp_buf + 16);
                    if (!netmask) netmask = 0xffffff00u;
                    puts_("net: "); putip(my_ip); puts_(" from DHCP\n");
                    return 0;
                }
                if (t == 6) break;
            }
    }
    error("no DHCP lease");
    return -1;
}

// ---- TFTP ----
static int tftp_ack(uint32_t srv, uint16_t lport, uint16_t tid, uint16_t blk)
{
    uint8_t a[4];
    put16(a, 4);  put16(a + 2, blk);
    return udp_send(srv, lport, tid, a, 4);
}

int net_load(const char *path, uint64_t addr, uint64_t max, uint64_t *size)
{
    if (net_init()) return -1;
    if (!my_ip && dhcp()) return -1;
    const uint32_t srv = TFTP_SERVER;
    const uint16_t lport = (uint16_t)(0xc000 | (now() & 0x3fff));
    uint32_t n = 2;
    put16(pkt, 1);
    for (const char *c = path; *c && n < 400; c++) pkt[n++] = (uint8_t)*c;
    pkt[n++] = 0;
    copy(pkt + n, "octet\0blksize\0" "1468", 18);  n += 18;  pkt[n++] = 0;
    uint32_t rrq_len = n;

    uint32_t blksize = 512;
    uint16_t tid = 0, expect = 1;
    uint64_t off = 0, t_start = now(), next_dot = 1 << 20;
    int tries = 0;
    if (udp_send(srv, lport, 69, pkt, rrq_len)) return -1;
    for (;;) {
        if (!udp_wait(lport, SEC)) {
            if (++tries > 5) { error("TFTP timed out"); return -1; }
            if (!tid) { if (udp_send(srv, lport, 69, pkt, rrq_len)) return -1; }
            else if (tftp_ack(srv, lport, tid, (uint16_t)(expect - 1))) return -1;
            continue;
        }
        if (udp_src != srv || udp_len < 4) continue;
        if (!tid) tid = udp_sport;
        else if (udp_sport != tid) continue;
        tries = 0;
        uint16_t op = be16(udp_buf);
        if (op == 5) {                                   // ERROR
            puts_("net: TFTP: ");
            udp_buf[udp_len < sizeof udp_buf ? udp_len : sizeof udp_buf - 1] = 0;
            puts_((const char *)udp_buf + 4);  putc_('\n');
            return be16(udp_buf + 2) == 1 ? -2 : -1;
        }
        if (op == 6) {                                   // OACK: the server took blksize
            for (uint32_t i = 2; i < udp_len; ) {
                const char *k = (const char *)udp_buf + i;
                uint32_t kl = 0;  while (i + kl < udp_len && k[kl]) kl++;
                const char *v = k + kl + 1;
                uint32_t vl = 0, x = 0;  while (i + kl + 1 + vl < udp_len && v[vl]) { x = x * 10 + (uint32_t)(v[vl] - '0'); vl++; }
                if (kl == 7 && same((const uint8_t *)k, (const uint8_t *)"blksize", 7) && x >= 8 && x <= BLKSIZE) blksize = x;
                i += kl + 1 + vl + 1;
            }
            if (tftp_ack(srv, lport, tid, 0)) return -1;
            continue;
        }
        if (op != 3) continue;
        uint16_t blk = be16(udp_buf + 2);
        uint32_t len = udp_len - 4;
        if (blk == expect) {
            if (off + len > max) { error("the file is larger than the space for it"); return -1; }
            copy((void *)(addr + off), udp_buf + 4, len);
            off += len;
            expect++;
            if (tftp_ack(srv, lport, tid, blk)) return -1;
            if (off >= next_dot) { putc_('.'); next_dot += 1 << 20; }
            if (len < blksize) break;
        } else if (blk == (uint16_t)(expect - 1)) {
            if (tftp_ack(srv, lport, tid, blk)) return -1;
        }
    }
    uint64_t ms = (now() - t_start) * 1000 / SEC;
    if (off >= 1 << 20) putc_('\n');
    puts_(path);  puts_(": ");  putdec(off);  puts_(" bytes in ");  putdec(ms);  puts_(" ms (blksize ");  putdec(blksize);  puts_(")\n");
    if (size) *size = off;
    return 0;
}
