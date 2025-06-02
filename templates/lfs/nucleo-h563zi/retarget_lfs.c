// Copyright (c) 2023 Cesanta Software Limited
// SPDX-License-Identifier: GPL-2.0-only or commercial

#include <sys/stat.h>

#include "hal.h"
#include "littlefs/lfs.h"

// This header must provide:
// FLASH_START_ADDR, FLASH_TOTAL_SIZE, FLASH_BLOCK_SIZE
// flash_write_buf(), flash_erase_block()
#include "flash_stm32h5.h"

#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

// Tunable constants
#define LFS_BLOCKS 16     // Number of blocks to use
#define LFS_BUF_SIZE 64   // Buffer size used for reads, writes, and cache
#define LFS_MAX_FDS 10    // Maximum number of opened files / dirs

#define MEM 0  // If 1, use ram FS. If 0, use flash
#if MEM
static uint8_t s_fs[FLASH_BLOCK_SIZE * LFS_BLOCKS];  // Keep FS in this memory
#else
// Allocate FS at the end of the flash memory
static uint8_t *s_fs = (uint8_t *) (FLASH_START_ADDR + FLASH_TOTAL_SIZE -
                                    FLASH_BLOCK_SIZE * LFS_BLOCKS);
#endif

static struct lfs_fd {
  int isopen;
  lfs_file_t file;
  lfs_dir_t dir;
} s_fds[LFS_MAX_FDS];

static int lfs_driver_read(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, void *buf, lfs_size_t len) {
  memmove(buf, &s_fs[block * cfg->block_size + off], len);
  // printf("%s(%p, %u, %u, %p, %u)\n", __func__, cfg, block, off, buf, len);
  return 0;
}

static int lfs_driver_prog(const struct lfs_config *cfg, lfs_block_t block,
                           lfs_off_t off, const void *buf, lfs_size_t len) {
  uint8_t *dst = &s_fs[block * cfg->block_size + off];
  (void) cfg;
  //printf("LFS> %s(%p,%lu,%lu,%p,%lu)\n", __func__, cfg, block, off, buf, len);
#if MEM
  memmove(dst, buf, len);
#else
  flash_write_buf(dst, buf, len);
  //printf("Flash write %lu bytes @ %p: %s. CR %#lx SR %#lx KEYR %#lx\n", len,
  //       dst, flash_is_err() ? "fail" : "ok", MG_REG(FLASH_NSCR),
  //       MG_REG(FLASH_NSSR), MG_REG(FLASH_KEYR));
#endif
  return 0;
}

static int lfs_driver_erase(const struct lfs_config *cfg, lfs_block_t block) {
  uint8_t *dst = &s_fs[block * cfg->block_size];
  // printf("LFS> %s(%p, block %lu, addr %p)\n", __func__, cfg, block, fs);
#if MEM
  memset(dst, 0xff, cfg->block_size);
#else
  flash_erase_block(dst);
  //printf("Erased: %d\n", ok);
#endif
  return 0;
}

static int lfs_driver_sync(const struct lfs_config *cfg) {
  //printf("%s(%p)\n", __func__, cfg);
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

int lfs_driver_init() {
  int result = 0;
  memset(&s_fds, 0, sizeof(s_fds));
  //printf("initializing... %d\n", flash_bank_is_swapped());
  if (lfs_mount(&s_lfs, &s_cfg) != 0) {
    //printf("formatting...\n");
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
  // printf("LFS> %s(%s,%d,%ld)->%d %d\n", __func__, path, flags, mode, fd,
  // err);
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

int mkdir(const char *path, mode_t mode) {
  // printf("%s(%s, %u)\n", __func__, path, mode);
  (void) path, (void) mode;
  return lfs_mkdir(&s_lfs, path);
}
