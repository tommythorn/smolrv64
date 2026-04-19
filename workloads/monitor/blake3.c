// Minimal BLAKE3 (unkeyed, 32-byte output) for the smolrv64 monitor.
// Derived from the BLAKE3 reference implementation (CC0 / MIT / Apache-2.0).
// One-shot hashing only; no keyed mode, no derive-key, no extendable output.

#include "blake3.h"

typedef unsigned char      u8;
typedef unsigned int       u32;
typedef unsigned long      u64;
typedef unsigned long      sz;

#define BLOCK_LEN 64
#define CHUNK_LEN 1024

#define CHUNK_START (1u << 0)
#define CHUNK_END   (1u << 1)
#define PARENT      (1u << 2)
#define ROOT        (1u << 3)

static const u32 IV[8] = {
    0x6A09E667u, 0xBB67AE85u, 0x3C6EF372u, 0xA54FF53Au,
    0x510E527Fu, 0x9B05688Cu, 0x1F83D9ABu, 0x5BE0CD19u,
};

static const u8 MSG_PERM[16] = {
    2, 6, 3, 10, 7, 0, 4, 13, 1, 11, 12, 5, 9, 14, 15, 8
};

static u32 rotr32(u32 x, unsigned n) {
    return (x >> n) | (x << (32 - n));
}

static void g(u32 *s, int a, int b, int c, int d, u32 mx, u32 my) {
    s[a] = s[a] + s[b] + mx;
    s[d] = rotr32(s[d] ^ s[a], 16);
    s[c] = s[c] + s[d];
    s[b] = rotr32(s[b] ^ s[c], 12);
    s[a] = s[a] + s[b] + my;
    s[d] = rotr32(s[d] ^ s[a], 8);
    s[c] = s[c] + s[d];
    s[b] = rotr32(s[b] ^ s[c], 7);
}

static void round_(u32 *s, const u32 *m) {
    g(s, 0, 4,  8, 12, m[0],  m[1]);
    g(s, 1, 5,  9, 13, m[2],  m[3]);
    g(s, 2, 6, 10, 14, m[4],  m[5]);
    g(s, 3, 7, 11, 15, m[6],  m[7]);
    g(s, 0, 5, 10, 15, m[8],  m[9]);
    g(s, 1, 6, 11, 12, m[10], m[11]);
    g(s, 2, 7,  8, 13, m[12], m[13]);
    g(s, 3, 4,  9, 14, m[14], m[15]);
}

static void permute(u32 *m) {
    u32 t[16];
    int i;
    for (i = 0; i < 16; i++) t[i] = m[MSG_PERM[i]];
    for (i = 0; i < 16; i++) m[i] = t[i];
}

static u32 load32(const u8 *b) {
    return (u32)b[0] | ((u32)b[1] << 8) | ((u32)b[2] << 16) | ((u32)b[3] << 24);
}

static void store32(u8 *b, u32 x) {
    b[0] = (u8)x; b[1] = (u8)(x >> 8); b[2] = (u8)(x >> 16); b[3] = (u8)(x >> 24);
}

static void compress(
    const u32 cv[8], const u8 block[64],
    u64 counter, u32 block_len, u32 flags, u32 out[16])
{
    u32 state[16], m[16];
    int i;
    for (i = 0; i < 8; i++) state[i] = cv[i];
    for (i = 0; i < 4; i++) state[8 + i] = IV[i];
    state[12] = (u32)counter;
    state[13] = (u32)(counter >> 32);
    state[14] = block_len;
    state[15] = flags;
    for (i = 0; i < 16; i++) m[i] = load32(block + 4 * i);

    for (i = 0; i < 6; i++) { round_(state, m); permute(m); }
    round_(state, m);

    for (i = 0; i < 8; i++) {
        out[i]     = state[i]     ^ state[i + 8];
        out[i + 8] = state[i + 8] ^ cv[i];
    }
}

// Hash one chunk. If is_root, out[0..16) is the root output state; otherwise
// out[0..8) is the chunk CV.
static void hash_chunk(
    const u8 *data, sz len, u64 counter, u32 flags, int is_root, u32 out[16])
{
    u32 cv[8];
    u8  block[64];
    sz  offset = 0;
    int first = 1;
    int i;
    sz  k;

    for (i = 0; i < 8; i++) cv[i] = IV[i];

    if (len == 0) {
        for (i = 0; i < 64; i++) block[i] = 0;
        u32 f = flags | CHUNK_START | CHUNK_END;
        if (is_root) f |= ROOT;
        compress(cv, block, counter, 0, f, out);
        return;
    }

    while (offset < len) {
        sz bl = len - offset;
        if (bl > 64) bl = 64;
        for (i = 0; i < 64; i++) block[i] = 0;
        for (k = 0; k < bl; k++) block[k] = data[offset + k];
        int is_last = (offset + bl == len);
        u32 f = flags;
        if (first) f |= CHUNK_START;
        if (is_last) f |= CHUNK_END;
        if (is_last && is_root) f |= ROOT;
        u32 sout[16];
        compress(cv, block, counter, (u32)bl, f, sout);
        if (is_last && is_root) {
            for (i = 0; i < 16; i++) out[i] = sout[i];
            return;
        }
        for (i = 0; i < 8; i++) cv[i] = sout[i];
        first = 0;
        offset += bl;
    }
    for (i = 0; i < 8; i++) out[i] = cv[i];
}

static void parent_compress(
    const u32 left[8], const u32 right[8], u32 flags, int is_root, u32 out[16])
{
    u8 block[64];
    int i;
    for (i = 0; i < 8; i++) store32(block + 4 * i, left[i]);
    for (i = 0; i < 8; i++) store32(block + 32 + 4 * i, right[i]);
    u32 f = flags | PARENT;
    if (is_root) f |= ROOT;
    compress(IV, block, 0, 64, f, out);
}

// Chaining-value stack. At most 54 entries (one per bit of a 54-bit chunk count,
// which covers 2^54 * 1024 = 2^64 bytes of input). Kept in BSS to spare stack.
static u32 cv_stack[54][8];

void blake3_hash(const void *input, unsigned long len, unsigned char out[32])
{
    const u8 *data = (const u8 *)input;
    int stack_len = 0;
    u64 counter = 0;
    u64 offset = 0;
    u32 flags = 0;
    u32 out16[16];
    int i;

    if (len <= CHUNK_LEN) {
        hash_chunk(data, (sz)len, 0, flags, 1, out16);
        for (i = 0; i < 8; i++) store32(out + 4 * i, out16[i]);
        return;
    }

    // All chunks except the last.
    while (offset + CHUNK_LEN < len) {
        u32 tmp[16];
        hash_chunk(data + offset, CHUNK_LEN, counter, flags, 0, tmp);
        u32 cv[8];
        for (i = 0; i < 8; i++) cv[i] = tmp[i];

        u64 post = counter + 1;
        while ((post & 1) == 0 && stack_len > 0) {
            u32 merged[16];
            parent_compress(cv_stack[stack_len - 1], cv, flags, 0, merged);
            for (i = 0; i < 8; i++) cv[i] = merged[i];
            stack_len--;
            post >>= 1;
        }
        for (i = 0; i < 8; i++) cv_stack[stack_len][i] = cv[i];
        stack_len++;
        counter++;
        offset += CHUNK_LEN;
    }

    // Last (possibly partial, but at least 1-byte) chunk.
    u32 last_cv[8];
    {
        u32 tmp[16];
        hash_chunk(data + offset, (sz)(len - offset), counter, flags, 0, tmp);
        for (i = 0; i < 8; i++) last_cv[i] = tmp[i];
    }

    // Walk up the stack. Only the topmost merge carries ROOT.
    while (stack_len > 1) {
        u32 merged[16];
        parent_compress(cv_stack[stack_len - 1], last_cv, flags, 0, merged);
        for (i = 0; i < 8; i++) last_cv[i] = merged[i];
        stack_len--;
    }
    parent_compress(cv_stack[0], last_cv, flags, 1, out16);
    for (i = 0; i < 8; i++) store32(out + 4 * i, out16[i]);
}
