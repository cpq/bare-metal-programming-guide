// Copyright (c) 2024 Cesanta Software Limited
// All rights reserved

#include "hal.h"

#include <sys/stat.h>  // For _fstat()

__attribute__((weak))  int _fstat(int fd, struct stat *st) {
  (void) fd, (void) st;
  return -1;
}

__attribute__((weak)) void *_sbrk(int incr) {
  unsigned char *prev_heap;
  unsigned char *heap_end = (unsigned char *) ((size_t) &heap_end - 256);
  prev_heap = s_current_heap_end;
  // Check how much space we  got from the heap end to the stack end
  if (s_current_heap_end + incr > heap_end) return (void *) -1;
  s_current_heap_end += incr;
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

__attribute__((weak)) void _init(void) {
}
