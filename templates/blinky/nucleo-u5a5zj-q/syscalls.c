#include <sys/stat.h>

#include "hal.h"

__attribute__((weak)) int _fstat(int fd, struct stat *st) {
  if (fd < 0) return -1;
  st->st_mode = S_IFCHR;
  return 0;
}

__attribute__((weak)) void *_sbrk(int incr) {
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;
  if (heap == NULL) heap = &_end;
  prev_heap = heap;
  heap += incr;
  return prev_heap;
}

__attribute__((weak)) int _open(const char *path) {
  (void) path;
  return -1;
}

__attribute__((weak)) int _close(int fd) {
  (void) fd;
  return -1;
}

__attribute__((weak)) int _isatty(int fd) {
  (void) fd;
  return 1;
}

__attribute__((weak)) int _lseek(int fd, int ptr, int dir) {
  (void) fd, (void) ptr, (void) dir;
  return 0;
}

__attribute__((weak)) void _exit(int status) {
  (void) status;
  for (;;) asm volatile("BKPT #0");
}

__attribute__((weak)) void _kill(int pid, int sig) {
  (void) pid, (void) sig;
}

__attribute__((weak)) int _getpid(void) {
  return -1;
}

__attribute__((weak)) int _write(int fd, char *ptr, int len) {
  (void) fd, (void) ptr, (void) len;
  return -1;
}

__attribute__((weak)) int _read(int fd, char *ptr, int len) {
  (void) fd, (void) ptr, (void) len;
  return -1;
}

__attribute__((weak)) int _link(const char *a, const char *b) {
  (void) a, (void) b;
  return -1;
}

__attribute__((weak)) int _unlink(const char *a) {
  (void) a;
  return -1;
}

__attribute__((weak)) int _stat(const char *path, struct stat *st) {
  (void) path, (void) st;
  return -1;
}

__attribute__((weak)) int mkdir(const char *path, mode_t mode) {
  (void) path, (void) mode;
  return -1;
}

__attribute__((weak)) void _init(void) {
}
