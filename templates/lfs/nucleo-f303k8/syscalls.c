// Copyright (c) 2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include <sys/stat.h>

#include "hal.h"
#include "lfs.h"

#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define LFS_BLOCKS 8     // Number of blocks to use
#define LFS_BUF_SIZE 64  // Buffer size used for reads, writes, and cache

static struct lfs_opened {
  struct lfs_opened *next;
  union {
    lfs_file_t file;
    lfs_dir_t dir;
  } u;
} * s_opened;  // List of opened files. Descriptor == u.file.id + 2

extern void *_eflash;  // End of flash

#ifndef LFS_USE_RAM
#define LFS_USE_RAM 0  // If 1, use ram FS. If 0, use flash
#endif

#if LFS_USE_RAM
#undef FLASH_BLOCK_SIZE
#define FLASH_BLOCK_SIZE 256
static uint8_t s_fs[FLASH_BLOCK_SIZE * LFS_BLOCKS];  // Keep FS in this memory
#else
// Allocate FS at the end of the flash memory
static uint8_t *s_fs = ((uint8_t *) &_eflash) - FLASH_BLOCK_SIZE * LFS_BLOCKS;
#endif

static inline char nibble(char c) {
  return c < 10 ? c + '0' : c + 'W';
}

static inline void hexdump(const void *buf, size_t len) {
  const uint8_t *p = (const uint8_t *) buf;
  char ascii[16];
  size_t i, j, n = 0;
  for (i = 0; i < len; i++) {
    if ((i % 16) == 0) {
      // Print buffered ascii chars
      if (i > 0) {
        putchar(' '), putchar(' ');
        for (j = 0; j < sizeof(ascii); j++) putchar(ascii[j]);
        putchar('\n'), n = 0;
      }
      // Print hex address, then \t
      putchar(nibble((i >> 12) & 15)), putchar(nibble((i >> 8) & 15));
      putchar(nibble((i >> 4) & 15)), putchar('0');
      putchar(' '), putchar(' '), putchar(' ');
    }
    putchar(nibble(p[i] >> 4)), putchar(nibble(p[i] & 15));
    putchar(' ');  // Space after hex number
    if (p[i] >= ' ' && p[i] <= '~') {
      ascii[n++] = (char) p[i];  // Printable
    } else {
      ascii[n++] = '.';  // Non-printable
    }
  }
  if (n > 0) {
    while (n < 16) putchar(' '), putchar(' '), putchar(' '), ascii[n++] = ' ';
    putchar(' '), putchar(' ');
    for (j = 0; j < sizeof(ascii); j++) putchar(ascii[j]);
  }
  putchar('\n');
}

static int lfs_driver_read(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, void *buf, lfs_size_t len) {
  memmove(buf, &s_fs[block * cfg->block_size + off], len);
  // printf("%s(%p,%lu,%lu,%p,%lu)\n", __func__, cfg, block, off, buf, len);
  return 0;
}

static int lfs_driver_prog(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, const void *buf, lfs_size_t len) {
  int result = 0;
  uint8_t *fs = &s_fs[block * cfg->block_size + off];
  // printf("LFS> %s(%p,%lu,%lu,%lu)\n", __func__, cfg, block, off, len);
#if LFS_USE_RAM
  memmove(&s_fs[block * cfg->block_size + off], buf, len);
#else
  if (!flash_write(buf, len, fs)) result = -1;
#endif
  if (result != 0) hexdump(fs, 64), printf("  ERR SR: %#lx\n", FLASH->SR);
  return result;
}

static int lfs_driver_erase(const struct lfs_config *cfg, lfs_block_t block) {
  int result = 0;
  uint8_t *fs = &s_fs[block * cfg->block_size];
  // printf("LFS> %s(%p, block %lu, addr %p)\n", __func__, cfg, block, fs);
#if LFS_USE_RAM
  memset(&s_fs[block * cfg->block_size], 0xff, cfg->block_size);
#else
  uint32_t pageno = FLASH_MAX_BLOCKS - LFS_BLOCKS + block;  // Page to erase
  if (!flash_erase(pageno)) result = -1;
#endif
  if (flash_is_err()) hexdump(fs, 64), printf("  ERR SR: %#lx\n", FLASH->SR);
  return result;
}

static int lfs_driver_sync(const struct lfs_config *cfg) {
  // printf("LFS> %s(%p)\n", __func__, cfg);
  (void) cfg;
  return 0;
}

static lfs_t s_lfs;
static const struct lfs_config s_cfg = {
    .read = lfs_driver_read,
    .prog = lfs_driver_prog,
    .erase = lfs_driver_erase,
    .sync = lfs_driver_sync,
    .block_size = FLASH_BLOCK_SIZE,
    .block_count = LFS_BLOCKS,
    .block_cycles = 200,
    .cache_size = LFS_BUF_SIZE,
    .read_size = LFS_BUF_SIZE,
    .prog_size = LFS_BUF_SIZE,
    .lookahead_size = LFS_BUF_SIZE / 8,
};

static int lfs_driver_init(void) {
  static int result = -1;
  if (result == -1) {
      lfs_format(&s_lfs, &s_cfg);
    if (lfs_mount(&s_lfs, &s_cfg) == 0) {
      result = 0;
    } else {
      lfs_format(&s_lfs, &s_cfg);
      if (lfs_mount(&s_lfs, &s_cfg) == 0) result = 0;
    }
  }
  return result;
}

static void closeopened(struct lfs_opened *p) {
  struct lfs_opened **head = &s_opened;
  while (*head && *head != p) head = &(*head)->next;
  if (*head) *head = p->next, free(p);
}

struct lfs_opened *openopened(void) {
  struct lfs_opened *p = calloc(1, sizeof(*p));
  if (p != NULL) p->next = s_opened, s_opened = p;
  return p;
}

DIR *opendir(const char *name) {
  lfs_driver_init();
  struct lfs_opened *p = openopened();
  if (p != NULL && lfs_dir_open(&s_lfs, &p->u.dir, name) != 0) {
    closeopened(p);
    p = NULL;
  }
  // printf("LFS> %s(%s) %p\n", __func__, name, p);
  return (DIR *) p;
}

int closedir(DIR *dir) {
  struct lfs_opened *p = (struct lfs_opened *) dir;
  lfs_dir_close(&s_lfs, &p->u.dir);
  closeopened(p);
  // printf("LFS> %s(%p)\n", __func__, p);
  return 0;
}

struct dirent *readdir(DIR *dir) {
  static struct dirent dirent;
  struct lfs_opened *p = (struct lfs_opened *) dir;
  struct lfs_info info = {};
  if (lfs_dir_read(&s_lfs, &p->u.dir, &info) < 1) return NULL;
  memset(&dirent, 0, sizeof(dirent));
  strncpy(dirent.d_name, info.name, sizeof(dirent.d_name) - 1);
  if (info.type == LFS_TYPE_DIR) dirent.d_type |= DT_DIR;
  if (info.type == LFS_TYPE_REG) dirent.d_type |= DT_REG;
  return &dirent;
}

int _open(const char *path, int flags, mode_t mode) {
  int err = 0, lfs_flags = 0, fd = -1;
  lfs_driver_init();
  if ((flags & 3) == O_RDONLY) lfs_flags |= LFS_O_RDONLY;
  if ((flags & 3) == O_WRONLY) lfs_flags |= LFS_O_WRONLY;
  if ((flags & 3) == O_RDWR) lfs_flags |= LFS_O_RDWR;
  if (flags & O_CREAT) lfs_flags |= LFS_O_CREAT;
  if (flags & O_TRUNC) lfs_flags |= LFS_O_TRUNC;
  if (flags & O_APPEND) lfs_flags |= LFS_O_APPEND;
  struct lfs_opened *p = openopened();
  if (p != NULL) {
    if ((err = lfs_file_open(&s_lfs, &p->u.file, path, lfs_flags)) < 0) {
      closeopened(p);
    } else {
      fd = p->u.file.id + 3;
    }
  }
  // printf("LFS> %s(%s,%d,%ld) %d %d\n", __func__, path, flags, mode, fd, err);
  (void) mode;
  return fd;
}

static struct lfs_opened *findbyfd(int fd) {
  struct lfs_opened *res = NULL;
  for (struct lfs_opened *p = s_opened; p != NULL && res == NULL; p = p->next) {
    if (p->u.file.id + 3 == fd) res = p;
  }
  // printf("LFS> %s(%d) -> %p\n", __func__, fd, res);
  return res;
};

int _close(int fd) {
  struct lfs_opened *p = findbyfd(fd);
  if (p != NULL) lfs_file_close(&s_lfs, &p->u.file);
  closeopened(p);
  // printf("LFS> %s(%d) -> %d\n", __func__, fd, result);
  return 0;
}

int _write(int fd, char *ptr, int len) {
  if (fd == 1 || fd == 2) uart_write_buf(UART_DEBUG, ptr, len);
  if (fd < 3) {
    return len;
  } else {
    struct lfs_opened *p = findbyfd(fd);
    return p == NULL ? -1 : lfs_file_write(&s_lfs, &p->u.file, ptr, len);
  }
}

int _read(int fd, char *ptr, int len) {
  if (fd < 3) {
    return len;
  } else {
    struct lfs_opened *p = findbyfd(fd);
    return p == NULL ? -1 : lfs_file_read(&s_lfs, &p->u.file, ptr, len);
  }
}

int _lseek(int fd, int offset, int whence) {
  struct lfs_opened *p = findbyfd(fd);
  return p == NULL ? -1 : lfs_file_seek(&s_lfs, &p->u.file, offset, whence);
}

int _rename(const char *oldname, const char *newname) {
  return lfs_rename(&s_lfs, oldname, newname);
}

int _unlink_r(void *r, const char *a) {
  (void) r;
  return lfs_remove(&s_lfs, a);
}

int _fstat(int fd, struct stat *st) {
  (void) fd, (void) st;
  return -1;
}

void *_sbrk(int incr) {
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;
  unsigned char x = 0, *heap_end = (unsigned char *) ((size_t) &x - 512);
  (void) x;
  if (heap == NULL) heap = (unsigned char *) &_end;  // Declared in hal.h
  prev_heap = heap;
  if (heap + incr > heap_end) return (void *) -1;
  heap += incr;
  return prev_heap;
}

int _isatty(int fd) {
  (void) fd;
  return 1;
}

void _exit(int status) {
  (void) status;
  for (;;) asm volatile("BKPT #0");
}

void _kill(int pid, int sig) {
  (void) pid, (void) sig;
}

int _getpid(void) {
  return -1;
}

int _link(const char *a, const char *b) {
  (void) a, (void) b;
  return -1;
}

int _unlink(const char *a) {
  (void) a;
  return -1;
}

int _stat(const char *path, struct stat *st) {
  (void) path, (void) st;
  return -1;
}

int mkdir(const char *path, mode_t mode) {
  // printf("%s(%s, %u)\n", __func__, path, mode);
  (void) path, (void) mode;
  return lfs_mkdir(&s_lfs, path);
}

void _init(void) {
}
