typedef unsigned int uint32_t;
void sha256_init(uint32_t st[8]);
void sha256_blocks(uint32_t st[8], const unsigned char *data, unsigned long num);
void sha256_final(uint32_t st[8], unsigned long total_bytes, unsigned char out[32]);
