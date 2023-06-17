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

// Constants defined by the TRM, section 3.3.1
#define FLASH_BLOCK_SIZE 2048  // FLash block size in bytes
#define FLASH_MAX_BLOCKS 128   // Total number of available blocks

// Tunable constants
#define LFS_BLOCKS 8     // Number of blocks to use
#define LFS_BUF_SIZE 64  // Buffer size used for reads, writes, and cache
#define LFS_MAX_FDS 6    // Maximum number of opened files / dirs

static struct lfs_fd {
  int isopen;
  lfs_file_t file;
  lfs_dir_t dir;
} s_fds[LFS_MAX_FDS];

extern void *_eflash;  // End of flash

static bool is_flash_err(void) {
  return FLASH->SR & ((BIT(14) - 1) << 2);  // TRM 3.7.5
}

static void flash_unlock(void) {
  static bool unlocked = false;
  if (!unlocked) {
    FLASH->KEYR = 0x45670123;  // Unlock
    spin(10);                  // Wait a bit
    FLASH->KEYR = 0xCDEF89AB;  // Flash
    unlocked = true;
  }
  FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_MER1);
  FLASH->CR &= ~(FLASH_CR_PNB | FLASH_CR_FSTPG);  // Clear Page addr, fast prog
  FLASH->SR = ~0UL;                               // Clear any previous errors
}

#define MEM 0  // If 1, use ram FS. If 0, use flash
#if MEM
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
  // printf("%s(%p, %u, %u, %p, %u)\n", __func__, cfg, block, off, buf, len);
  return 0;
}

static int lfs_driver_prog(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, const void *buf, lfs_size_t len) {
  int result = 0;
  uint8_t *fs = &s_fs[block * cfg->block_size + off];
  uint32_t *dst = (uint32_t *) fs;
  uint32_t *src = (uint32_t *) buf, *end = (uint32_t *) ((char *) buf + len);
  printf("%s(%p, %lu, %lu, %p, %lu)\n", __func__, cfg, block, off, buf, len);
#if MEM
  memmove(&s_fs[block * cfg->block_size + off], buf, len);
  return 0;
#endif
  while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
  flash_unlock();
  FLASH->CR |= FLASH_CR_PG;
  // printf("Writing %p, CR %#lx SR %#lx ...\n", dst, FLASH->CR, FLASH->SR);
  while (result == 0 && src < end) {
    *dst++ = *src++;
    *dst++ = *src++;
    while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
    if (is_flash_err()) result = -1;
  }
  if (result != 0) hexdump(fs, 64), printf("  ERR SR: %#lx\n", FLASH->SR);
  return result;
}

static int lfs_driver_erase(const struct lfs_config *cfg, lfs_block_t block) {
  uint8_t *fs = &s_fs[block * cfg->block_size];
  printf("%s(%p, block %lu, addr %p)\n", __func__, cfg, block, fs);
#if MEM
  memset(&s_fs[block * cfg->block_size], 0xff, cfg->block_size);
  return 0;
#endif
  while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until flash is not busy
  uint32_t page = FLASH_MAX_BLOCKS - LFS_BLOCKS + block;  // Page to erase
  flash_unlock();
  FLASH->CR |= FLASH_CR_PER | (page << 3);  // Set PER bit and page no
  FLASH->CR |= FLASH_CR_STRT;               // Start erasing
  printf("Erasing page %lu, CR %#lx SR %#lx ...\n", page, FLASH->CR, FLASH->SR);
  while (FLASH->SR & FLASH_SR_BSY) spin(1);  // Wait until erased
  if (is_flash_err()) hexdump(fs, 64), printf("  ERR SR: %#lx\n", FLASH->SR);
  return 0;
}

static int lfs_driver_sync(const struct lfs_config *cfg) {
  printf("%s(%p)\n", __func__, cfg);
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

int lfs_driver_init() {
  int result = 0;
  memset(&s_fds, 0, sizeof(s_fds));
  if (lfs_mount(&s_lfs, &s_cfg) != 0) {
    lfs_format(&s_lfs, &s_cfg);
    if (lfs_mount(&s_lfs, &s_cfg) != 0) result = -1;
  }
  return result;
}

static int open_fd(void) {
  static int initialized = 0;
  int i, fd = -1;
  if (!initialized && (lfs_driver_init() == 0)) initialized = 1;
  if (initialized) {
    for (i = 0; i < LFS_MAX_FDS; i++) {
      if (s_fds[i].isopen == 0) {
        s_fds[i].isopen = 1;
        fd = i + 3;
        break;
      }
    }
  }
  // printf("%s called, fd=%d\n", __func__, fd);
  return fd;
}

static int close_fd(int fd) {
  s_fds[fd - 3].isopen = 0;
  return 0;
}

int _open(const char *path, int flags, mode_t mode) {
  int err, lfs_flags = 0, fd = open_fd();
  (void) mode;
  if (fd == -1) return -1;
  if ((flags & 3) == O_RDONLY) lfs_flags |= LFS_O_RDONLY;
  if ((flags & 3) == O_WRONLY) lfs_flags |= LFS_O_WRONLY;
  if ((flags & 3) == O_RDWR) lfs_flags |= LFS_O_RDWR;
  if (flags & O_CREAT) lfs_flags |= LFS_O_CREAT;
  if (flags & O_TRUNC) lfs_flags |= LFS_O_TRUNC;
  if (flags & O_APPEND) lfs_flags |= LFS_O_APPEND;
  err = lfs_file_open(&s_lfs, &s_fds[fd - 3].file, path, lfs_flags);
  if (err < 0) close_fd(fd), fd = -1;
  printf("%s(%s, %d, %ld)->%d, err %d\n", __func__, path, flags, mode, fd, err);
  return fd;
}

int _close(int fd) {
  // printf("%s called. fd %d\n", __func__, fd);
  if (fd > 2) lfs_file_close(&s_lfs, &s_fds[fd - 3].file), close_fd(fd);
  return 0;
}

int _write(int fd, char *ptr, int len) {
  if (fd == 1 || fd == 2) uart_write_buf(UART_DEBUG, ptr, len);
  return fd < 3 ? len : lfs_file_write(&s_lfs, &s_fds[fd - 3].file, ptr, len);
}

int _read(int fd, char *ptr, int len) {
  return fd < 3 ? 0 : lfs_file_read(&s_lfs, &s_fds[fd - 3].file, ptr, len);
}

int _lseek(int fd, int offset, int whence) {
  return lfs_file_seek(&s_lfs, &s_fds[fd - 3].file, offset, whence);
}

int _rename(const char *oldname, const char *newname) {
  return lfs_rename(&s_lfs, oldname, newname);
}

int _unlink_r(void *r, const char *a) {
  (void) r;
  return lfs_remove(&s_lfs, a);
}

DIR *opendir(const char *name) {
  int fd = open_fd();
  if (fd == -1) return NULL;
  if (lfs_dir_open(&s_lfs, &s_fds[fd - 3].dir, name) != 0) {
    close_fd(fd);
    return NULL;
  }
  return (DIR *) &s_fds[fd - 3];
}

int closedir(DIR *dir) {
  struct lfs_fd *f = (struct lfs_fd *) dir;
  lfs_dir_close(&s_lfs, &f->dir);
  f->isopen = 0;
  return 0;
}

struct dirent *readdir(DIR *dir) {
  static struct dirent dirent;
  struct lfs_fd *f = (struct lfs_fd *) dir;
  struct lfs_info info;
  if (lfs_dir_read(&s_lfs, &f->dir, &info) < 1) return NULL;
  memset(&dirent, 0, sizeof(dirent));
  strncpy(dirent.d_name, info.name, sizeof(dirent.d_name) - 1);
  if (info.type == LFS_TYPE_DIR) dirent.d_type |= DT_DIR;
  if (info.type == LFS_TYPE_REG) dirent.d_type |= DT_REG;
  return &dirent;
}

int _fstat(int fd, struct stat *st) {
  if (fd < 0 && fd > LFS_MAX_FDS + 2) return -1;
  st->st_mode = S_IFCHR;
  return 0;
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
