// sha256.c -- SHA-256 in the shape of OpenSSL's C fallback (crypto/sha/sha256.c without
// SHA256_ASM): sixteen rounds written out, then a loop of eight-round bodies, X[16] as a
// ring, rotates as shift pairs (this core has no Zbb).  This is the code the board's
// coreutils sha256sum runs through libcrypto.so.3, so a stack measured on it is a stack
// of the same instruction mix, minus the read(2) path.
#include "sha256.h"

static const uint32_t K256[64] = {
   0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
   0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
   0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
   0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
   0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
   0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
   0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
   0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2 };

#define ROTATE(x,n)  (((x) << (n)) | ((x) >> (32 - (n))))
#define Sigma0(x)    (ROTATE((x),30) ^ ROTATE((x),19) ^ ROTATE((x),10))
#define Sigma1(x)    (ROTATE((x),26) ^ ROTATE((x),21) ^ ROTATE((x),7))
#define sigma0(x)    (ROTATE((x),25) ^ ROTATE((x),14) ^ ((x) >> 3))
#define sigma1(x)    (ROTATE((x),15) ^ ROTATE((x),13) ^ ((x) >> 10))
#define Ch(x,y,z)    (((x) & (y)) ^ ((~(x)) & (z)))
#define Maj(x,y,z)   (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define ROUND_00_15(i,a,b,c,d,e,f,g,h) do { \
   T1 += h + Sigma1(e) + Ch(e,f,g) + K256[i]; \
   h = Sigma0(a) + Maj(a,b,c); \
   d += T1; h += T1; } while (0)
#define ROUND_16_63(i,a,b,c,d,e,f,g,h,X) do { \
   s0 = X[(i+1)&0x0f]; s0 = sigma0(s0); \
   s1 = X[(i+14)&0x0f]; s1 = sigma1(s1); \
   T1 = X[(i)&0x0f] += s0 + s1 + X[(i+9)&0x0f]; \
   ROUND_00_15(i,a,b,c,d,e,f,g,h); } while (0)

void sha256_blocks(uint32_t st[8], const unsigned char *data, unsigned long num)
{
   uint32_t a, b, c, d, e, f, g, h, s0, s1, T1;
   uint32_t X[16];
   int i;
   while (num--) {
      a = st[0]; b = st[1]; c = st[2]; d = st[3];
      e = st[4]; f = st[5]; g = st[6]; h = st[7];
#define HOST_c2l(p,l) (l = ((uint32_t)(p)[0] << 24) | ((uint32_t)(p)[1] << 16) | \
                           ((uint32_t)(p)[2] << 8) | (uint32_t)(p)[3], (p) += 4)
      (void)HOST_c2l(data, T1); X[0]  = T1; ROUND_00_15(0,  a,b,c,d,e,f,g,h);
      (void)HOST_c2l(data, T1); X[1]  = T1; ROUND_00_15(1,  h,a,b,c,d,e,f,g);
      (void)HOST_c2l(data, T1); X[2]  = T1; ROUND_00_15(2,  g,h,a,b,c,d,e,f);
      (void)HOST_c2l(data, T1); X[3]  = T1; ROUND_00_15(3,  f,g,h,a,b,c,d,e);
      (void)HOST_c2l(data, T1); X[4]  = T1; ROUND_00_15(4,  e,f,g,h,a,b,c,d);
      (void)HOST_c2l(data, T1); X[5]  = T1; ROUND_00_15(5,  d,e,f,g,h,a,b,c);
      (void)HOST_c2l(data, T1); X[6]  = T1; ROUND_00_15(6,  c,d,e,f,g,h,a,b);
      (void)HOST_c2l(data, T1); X[7]  = T1; ROUND_00_15(7,  b,c,d,e,f,g,h,a);
      (void)HOST_c2l(data, T1); X[8]  = T1; ROUND_00_15(8,  a,b,c,d,e,f,g,h);
      (void)HOST_c2l(data, T1); X[9]  = T1; ROUND_00_15(9,  h,a,b,c,d,e,f,g);
      (void)HOST_c2l(data, T1); X[10] = T1; ROUND_00_15(10, g,h,a,b,c,d,e,f);
      (void)HOST_c2l(data, T1); X[11] = T1; ROUND_00_15(11, f,g,h,a,b,c,d,e);
      (void)HOST_c2l(data, T1); X[12] = T1; ROUND_00_15(12, e,f,g,h,a,b,c,d);
      (void)HOST_c2l(data, T1); X[13] = T1; ROUND_00_15(13, d,e,f,g,h,a,b,c);
      (void)HOST_c2l(data, T1); X[14] = T1; ROUND_00_15(14, c,d,e,f,g,h,a,b);
      (void)HOST_c2l(data, T1); X[15] = T1; ROUND_00_15(15, b,c,d,e,f,g,h,a);
      for (i = 16; i < 64; i += 8) {
         ROUND_16_63(i + 0, a,b,c,d,e,f,g,h, X);
         ROUND_16_63(i + 1, h,a,b,c,d,e,f,g, X);
         ROUND_16_63(i + 2, g,h,a,b,c,d,e,f, X);
         ROUND_16_63(i + 3, f,g,h,a,b,c,d,e, X);
         ROUND_16_63(i + 4, e,f,g,h,a,b,c,d, X);
         ROUND_16_63(i + 5, d,e,f,g,h,a,b,c, X);
         ROUND_16_63(i + 6, c,d,e,f,g,h,a,b, X);
         ROUND_16_63(i + 7, b,c,d,e,f,g,h,a, X);
      }
      st[0] += a; st[1] += b; st[2] += c; st[3] += d;
      st[4] += e; st[5] += f; st[6] += g; st[7] += h;
   }
}

void sha256_init(uint32_t st[8])
{
   st[0] = 0x6a09e667; st[1] = 0xbb67ae85; st[2] = 0x3c6ef372; st[3] = 0xa54ff53a;
   st[4] = 0x510e527f; st[5] = 0x9b05688c; st[6] = 0x1f83d9ab; st[7] = 0x5be0cd19;
}

// The message is `reps` copies of a 64-byte-multiple buffer, so the padding is exactly one
// final block: 0x80, zeros, and the 64-bit bit length.
void sha256_final(uint32_t st[8], unsigned long total_bytes, unsigned char out[32])
{
   unsigned char pad[64];
   unsigned long bits = total_bytes * 8;
   int i;
   for (i = 0; i < 64; i++) pad[i] = 0;
   pad[0] = 0x80;
   for (i = 0; i < 8; i++) pad[56 + i] = (unsigned char)(bits >> (56 - 8 * i));
   sha256_blocks(st, pad, 1);
   for (i = 0; i < 8; i++) {
      out[4*i]   = (unsigned char)(st[i] >> 24); out[4*i+1] = (unsigned char)(st[i] >> 16);
      out[4*i+2] = (unsigned char)(st[i] >> 8);  out[4*i+3] = (unsigned char)(st[i]);
   }
}
