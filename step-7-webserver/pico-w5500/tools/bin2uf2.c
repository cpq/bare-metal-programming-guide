// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fail(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  putchar('\n');
  exit(EXIT_FAILURE);
}

static void wu32(FILE *out, uint32_t value) {
  fwrite(&value, 1, sizeof(value), out);
}

static void addcrc(uint8_t *buf) {
  uint32_t crc = ~0U;
  for (int j = 0; j < 252; j++) {
    uint32_t b = (uint32_t) buf[j] << 24;
    for (int i = 0; i < 8; i++) {
      crc = (crc << 1) ^ (((crc ^ b) & 0x80000000) ? 0X04c11db7U : 0);
      b <<= 1;
    }
  }
  memcpy(buf + 252, &crc, 4);
}

int main(int argc, char *argv[]) {
  FILE *in, *out;

  if (argc != 3) fail("Usage: bin2uf2 FILE.bin FILE.uf2\n");
  if ((in = fopen(argv[1], "rb")) == NULL) fail("open(%s)\n", argv[1]);
  if ((out = fopen(argv[2], "w+b")) == NULL) fail("open(%s)\n", argv[2]);

  // Find input file size, and number of blocks
  fseek(in, 0, SEEK_END);
  size_t size = ftell(in), nblocks = (size + 255) / 256;
  fseek(in, 0, SEEK_SET);

  char buf[256];
  size_t n, blockno = 0;
  while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
    if (n < sizeof(buf)) memset(buf + n, 0xff, sizeof(buf) - n);
    if (blockno == 0) addcrc((uint8_t *) buf);
    wu32(out, 0x0a324655);                          // MAGIC_START_0
    wu32(out, 0x9e5d5157);                          // MAGIC_START_1
    wu32(out, 0x00002000);                          // Flags
    wu32(out, 0x10000000 + blockno * sizeof(buf));  // Address
    wu32(out, sizeof(buf));                         // Size
    wu32(out, blockno);                             // Block number
    wu32(out, nblocks);                             // Total blocks
    wu32(out, 0xe48bff56);                          // Family ID
    fwrite(buf, 1, sizeof(buf), out);               // Data
    for (int i = 0; i < 55; i++) wu32(out, 0);      // Remaining 220 bytes
    wu32(out, 0x0ab16f30);                          // MAGIC_END
    blockno++;
  }

  printf("Written %d bytes into %s\n", (int) ftell(out), argv[2]);
  fclose(in);
  fclose(out);

  return EXIT_SUCCESS;
}
