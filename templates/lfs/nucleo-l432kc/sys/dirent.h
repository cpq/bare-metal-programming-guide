#pragma once
#include <stdio.h>

#define DT_DIR 4
#define DT_REG 8

struct dirent {
  char d_name[FILENAME_MAX];
  unsigned char d_type;
};

typedef struct {
  struct dirent result;
} DIR;
