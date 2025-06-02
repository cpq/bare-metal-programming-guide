// Copyright (c) 2022-2025 Cesanta Software Limited
// SPDX-License-Identifier: GPL-2.0-only or commercial
#pragma once

#define FLASH_START_ADDR 0x8000000
#define FLASH_TOTAL_SIZE (2 * 1024 * 1024)  // Defined in the CMSIS header
#define FLASH_BLOCK_SIZE 8192               // FLash block size in bytes
#define FLASH_ALIGN_SIZE 16

#define MG_REG(x) ((volatile uint32_t *) (x))[0]
#define MG_BIT(x) (((uint32_t) 1U) << (x))
#define MG_SET_BITS(R, CLRMASK, SETMASK) (R) = ((R) & ~(CLRMASK)) | (SETMASK)

#ifndef MG_DEBUG
#define MG_DEBUG(x)  // printf x; putchar('\n')
#define MG_ERROR(x)  // printf x; putchar('\n')
#endif

#if defined(__GNUC__)
#define MG_ARM_DISABLE_IRQ() asm volatile("cpsid i" : : : "memory")
#define MG_ARM_ENABLE_IRQ() asm volatile("cpsie i" : : : "memory")
#elif defined(__CCRH__)
#define MG_RH850_DISABLE_IRQ() __DI()
#define MG_RH850_ENABLE_IRQ() __EI()
#else
#define MG_ARM_DISABLE_IRQ()
#define MG_ARM_ENABLE_IRQ()
#endif

#define MG_FLASH_CONTROLLER_ADDR 0x40022000  // Flash controller base address
#define FLASH_KEYR (MG_FLASH_CONTROLLER_ADDR + 0x4)  // See RM0481 7.11
#define FLASH_OPTKEYR (MG_FLASH_CONTROLLER_ADDR + 0xc)
#define FLASH_OPTCR (MG_FLASH_CONTROLLER_ADDR + 0x1c)
#define FLASH_NSSR (MG_FLASH_CONTROLLER_ADDR + 0x20)
#define FLASH_NSCR (MG_FLASH_CONTROLLER_ADDR + 0x28)
#define FLASH_NSCCR (MG_FLASH_CONTROLLER_ADDR + 0x30)
#define FLASH_OPTSR_CUR (MG_FLASH_CONTROLLER_ADDR + 0x50)
#define FLASH_OPTSR_PRG (MG_FLASH_CONTROLLER_ADDR + 0x54)

static inline void flash_unlock(void) {
  if (MG_REG(FLASH_NSCR) & MG_BIT(0)) {
    MG_REG(FLASH_KEYR) = 0x45670123;
    MG_REG(FLASH_KEYR) = 0xcdef89ab;
    MG_REG(FLASH_OPTKEYR) = 0x08192a3b;
    MG_REG(FLASH_OPTKEYR) = 0x4c5d6e7f;
  }
}

static inline int flash_page_start(volatile uint32_t *dst) {
  char *base = (char *) FLASH_START_ADDR, *end = base + FLASH_TOTAL_SIZE;
  volatile char *p = (char *) dst;
  return p >= base && p < end && ((p - base) % FLASH_BLOCK_SIZE) == 0;
}

static inline bool flash_is_err(void) {
  return MG_REG(FLASH_NSSR) & ((MG_BIT(8) - 1) << 17);  // RM0481 7.11.9
}

static inline void flash_wait(void) {
  while ((MG_REG(FLASH_NSSR) & MG_BIT(0)) &&
         (MG_REG(FLASH_NSSR) & MG_BIT(16)) == 0) {
    (void) 0;
  }
}

static inline void flash_clear_err(void) {
  flash_wait();                                    // Wait until ready
  MG_REG(FLASH_NSCCR) = ((MG_BIT(9) - 1) << 16U);  // Clear all errors
}

static inline bool flash_bank_is_swapped(void) {
  return MG_REG(FLASH_OPTCR) & MG_BIT(31);  // RM0481 7.11.8
}

static inline bool flash_erase_block(void *location) {
  bool ok = false;
  if (flash_page_start(location) == false) {
    MG_ERROR(("%p is not on a sector boundary", location));
  } else {
    uintptr_t diff = (char *) location - (char *) FLASH_START_ADDR;
    uint32_t sector = diff / FLASH_BLOCK_SIZE;
    flash_unlock();
    flash_clear_err();
    MG_REG(FLASH_NSCR) = 0;
    if ((sector < 128 && flash_bank_is_swapped()) ||
        (sector > 127 && !flash_bank_is_swapped())) {
      MG_REG(FLASH_NSCR) |= MG_BIT(31);  // Set FLASH_CR_BKSEL
    }
    if (sector > 127) sector -= 128;
    MG_REG(FLASH_NSCR) |= MG_BIT(2) | (sector << 6);  // Erase | sector_num
    MG_REG(FLASH_NSCR) |= MG_BIT(5);                  // Start erasing
    flash_wait();
    ok = !flash_is_err();
    MG_DEBUG(("Erase sector %lu @ %p: %s. CR %#lx SR %#lx", sector, location,
              ok ? "ok" : "fail", MG_REG(FLASH_NSCR), MG_REG(FLASH_NSSR)));
    // mg_hexdump(location, 32);
    MG_REG(FLASH_NSCR) = 0;  // Restore saved CR
  }
  return ok;
}

static inline bool flash_swap_bank(void) {
  uint32_t desired = flash_bank_is_swapped() ? 0 : MG_BIT(31);
  flash_unlock();
  flash_clear_err();
  // printf("OPTSR_PRG 1 %#lx\n", FLASH->OPTSR_PRG);
  MG_SET_BITS(MG_REG(FLASH_OPTSR_PRG), MG_BIT(31), desired);
  // printf("OPTSR_PRG 2 %#lx\n", FLASH->OPTSR_PRG);
  MG_REG(FLASH_OPTCR) |= MG_BIT(1);  // OPTSTART
  while ((MG_REG(FLASH_OPTSR_CUR) & MG_BIT(31)) != desired) (void) 0;
  return true;
}

static inline size_t flash_write_buf(void *addr, const void *buf, size_t len) {
  if ((len % FLASH_ALIGN_SIZE) != 0) {
    MG_ERROR(("%u is not aligned to %u", len, FLASH_ALIGN_SIZE));
    return 0;
  }
  uint32_t *dst = (uint32_t *) addr;
  uint32_t *src = (uint32_t *) buf;
  uint32_t *end = (uint32_t *) ((char *) buf + len);
  bool ok = true;
  MG_ARM_DISABLE_IRQ();
  flash_unlock();
  flash_clear_err();
  MG_REG(FLASH_NSCR) = MG_BIT(1);  // Set programming flag
  while (ok && src < end) {
#if 0
    if (flash_page_start(dst) && flash_erase_block(dst) == false) {
      ok = false;
      break;
    }
#endif
    *(volatile uint32_t *) dst++ = *src++;
    flash_wait();
    if (flash_is_err()) {
      ok = false;
      break;
    }
  }
  MG_ARM_ENABLE_IRQ();
  MG_DEBUG(("Flash write %zu bytes @ %p: %s. CR %#lx SR %#lx", len, dst,
            flash_is_err() ? "fail" : "ok", MG_REG(FLASH_NSCR),
            MG_REG(FLASH_NSSR)));
  MG_REG(FLASH_NSCR) = 0;  // Restore CR
  return (size_t) ((char *) src - (char *) buf);
}
