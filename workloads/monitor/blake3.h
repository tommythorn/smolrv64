// Minimal BLAKE3 (unkeyed, 32-byte output) for the smolrv64 monitor.
// One-shot hash of a contiguous byte range in memory.

#ifndef BLAKE3_H
#define BLAKE3_H

void blake3_hash(const void *input, unsigned long len, unsigned char out[32]);

#endif
