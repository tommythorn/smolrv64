// shalib: the board's OWN libcrypto sha256 (what coreutils sha256sum runs) on the shabench
// buffer, no read path, no EVP, no timer -- dlopen so the host cross-link needs nothing from
// the board.  Same message as shabench, so the digest checks against `make check`.
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#define BUF_BYTES 65536
static unsigned char buf[BUF_BYTES];
int main(int argc, char **argv)
{
   unsigned long reps = argc > 1 ? strtoul(argv[1], 0, 0) : 64, i, r;
   unsigned char ctx[512], out[32];
   void *h = dlopen("libcrypto.so.3", RTLD_NOW);
   if (!h) { fprintf(stderr, "dlopen: %s\n", dlerror()); return 1; }
   int (*init)(void *) = dlsym(h, "SHA256_Init");
   int (*update)(void *, const void *, unsigned long) = dlsym(h, "SHA256_Update");
   int (*final)(unsigned char *, void *) = dlsym(h, "SHA256_Final");
   if (!init || !update || !final) { fprintf(stderr, "dlsym failed\n"); return 1; }
   for (i = 0; i < BUF_BYTES; i++) buf[i] = (unsigned char)((i * 2654435761ul) >> 24);
   init(ctx);
   for (r = 0; r < reps; r++) update(ctx, buf, BUF_BYTES);
   final(out, ctx);
   for (i = 0; i < 32; i++) printf("%02x", out[i]);
   printf("  reps=%lu\n", reps);
   return 0;
}
