
#include <mutek/printk.h>
#include <hexo/power.h>
#include <hexo/bit.h>
#include <assert.h>

volatile uint8_t ex = 0;

#define TEST(op, a, r) do {                             \
  assert(op(a) == r);           /* constant */          \
  assert(op( (typeof(a)) ((a) | ex) ) == r); /* variable */    \
} while (0);

void app_start()
{
  TEST(bit_clz8, 0x80, 0);
  TEST(bit_clz8, 0x08, 4);
  TEST(bit_clz8, 0x0f, 4);
  TEST(bit_clz8, 0x01, 7);
  TEST(bit_clz8, 0x180, 0);
  TEST(bit_clz8, 0x108, 4);

  TEST(bit_clz_unsafe, (uint8_t)0x80, 0);
  TEST(bit_clz_unsafe, (uint8_t)0x08, 4);
  TEST(bit_clz_unsafe, (uint8_t)0x0f, 4);
  TEST(bit_clz_unsafe, (uint8_t)0x01, 7);

  TEST(bit_msb_index, (uint8_t)0x80, 7);
  TEST(bit_msb_index, (uint8_t)0x08, 3);
  TEST(bit_msb_index, (uint8_t)0x0f, 3);
  TEST(bit_msb_index, (uint8_t)0x01, 0);

  TEST(bit_clz16, 0x8000,  0);
  TEST(bit_clz16, 0x0800,  4);
  TEST(bit_clz16, 0x0fff,  4);
  TEST(bit_clz16, 0x0100,  7);
  TEST(bit_clz16, 0x0001,  15 );
  TEST(bit_clz16, 0x18000, 0);
  TEST(bit_clz16, 0x10800, 4);

  TEST(bit_clz_unsafe, (uint16_t)0x8000, 0);
  TEST(bit_clz_unsafe, (uint16_t)0x0800, 4);
  TEST(bit_clz_unsafe, (uint16_t)0x0fff, 4);
  TEST(bit_clz_unsafe, (uint16_t)0x0100, 7);
  TEST(bit_clz_unsafe, (uint16_t)0x0001, 15);

  TEST(bit_msb_index, (uint16_t)0x8000, 15);
  TEST(bit_msb_index, (uint16_t)0x0800, 11);
  TEST(bit_msb_index, (uint16_t)0x0fff, 11);
  TEST(bit_msb_index, (uint16_t)0x0100, 8);
  TEST(bit_msb_index, (uint16_t)0x0001, 0);

  TEST(bit_clz32, 0x80000000,  0);
  TEST(bit_clz32, 0x08000000,  4);
  TEST(bit_clz32, 0x0fff0000,  4);
  TEST(bit_clz32, 0x01000000,  7);
  TEST(bit_clz32, 0x00010000,  15 );
  TEST(bit_clz32, 0x00000001,  31 );
  TEST(bit_clz32, 0x180000000ULL, 0);
  TEST(bit_clz32, 0x108000000ULL, 4);

  TEST(bit_clz_unsafe, (uint32_t)0x80000000, 0);
  TEST(bit_clz_unsafe, (uint32_t)0x08000000, 4);
  TEST(bit_clz_unsafe, (uint32_t)0x0fff0000, 4);
  TEST(bit_clz_unsafe, (uint32_t)0x01000000, 7);
  TEST(bit_clz_unsafe, (uint32_t)0x00010000, 15);
  TEST(bit_clz_unsafe, (uint32_t)0x00000100, 23);
  TEST(bit_clz_unsafe, (uint32_t)0x00000001, 31);

  TEST(bit_msb_index, (uint32_t)0x80000000, 31);
  TEST(bit_msb_index, (uint32_t)0x08000000, 27);
  TEST(bit_msb_index, (uint32_t)0x0fff0000, 27);
  TEST(bit_msb_index, (uint32_t)0x01000000, 24);
  TEST(bit_msb_index, (uint32_t)0x00010000, 16);
  TEST(bit_msb_index, (uint32_t)0x00000100, 8);
  TEST(bit_msb_index, (uint32_t)0x00000001, 0);

  TEST(bit_clz64, 0x8000000000000000ULL,  0);
  TEST(bit_clz64, 0x0800000000000000ULL,  4);
  TEST(bit_clz64, 0x0fff000000000000ULL,  4);
  TEST(bit_clz64, 0x0100000000000000ULL,  7);
  TEST(bit_clz64, 0x0001000000000000ULL,  15 );
  TEST(bit_clz64, 0x0000000100000000ULL,  31 );
  TEST(bit_clz64, 0x0000000080000000ULL,  32 );
  TEST(bit_clz64, 0x0000000000000001ULL,  63 );

  TEST(bit_clz_unsafe, (uint64_t)0x8000000000000000ULL, 0);
  TEST(bit_clz_unsafe, (uint64_t)0x0800000000000000ULL, 4);
  TEST(bit_clz_unsafe, (uint64_t)0x0fff000000000000ULL, 4);
  TEST(bit_clz_unsafe, (uint64_t)0x0100000000000000ULL, 7);
  TEST(bit_clz_unsafe, (uint64_t)0x0001000000000000ULL, 15);
  TEST(bit_clz_unsafe, (uint64_t)0x0000010000000000ULL, 23);
  TEST(bit_clz_unsafe, (uint64_t)0x0000000100000000ULL, 31);
  TEST(bit_clz_unsafe, (uint64_t)0x0000000080000000ULL, 32);
  TEST(bit_clz_unsafe, (uint64_t)0x0000000000000001ULL, 63);

  TEST(bit_msb_index, (uint64_t)0x8000000000000000ULL, 63);
  TEST(bit_msb_index, (uint64_t)0x0800000000000000ULL, 59);
  TEST(bit_msb_index, (uint64_t)0x0fff000000000000ULL, 59);
  TEST(bit_msb_index, (uint64_t)0x0100000000000000ULL, 56);
  TEST(bit_msb_index, (uint64_t)0x0001000000000000ULL, 48);
  TEST(bit_msb_index, (uint64_t)0x0000010000000000ULL, 40);
  TEST(bit_msb_index, (uint64_t)0x0000000100000000ULL, 32);
  TEST(bit_msb_index, (uint64_t)0x0000000080000000ULL, 31);
  TEST(bit_msb_index, (uint64_t)0x0000000000000001ULL, 0);

  TEST(bit_ctz8, 0x80, 7);
  TEST(bit_ctz8, 0x08, 3);
  TEST(bit_ctz8, 0x0f, 0);
  TEST(bit_ctz8, 0xf0, 4);

  TEST(bit_ctz, (uint8_t)0x80, 7);
  TEST(bit_ctz, (uint8_t)0x08, 3);
  TEST(bit_ctz, (uint8_t)0x0f, 0);
  TEST(bit_ctz, (uint8_t)0xf0, 4);

  TEST(bit_ctz16, 0x80, 7);
  TEST(bit_ctz16, 0x08, 3);
  TEST(bit_ctz16, 0x0f, 0);
  TEST(bit_ctz16, 0xf0, 4);
  TEST(bit_ctz16, 0xf00, 8);
  TEST(bit_ctz16, 0x800, 11);

  TEST(bit_ctz, (uint16_t)0x80, 7);
  TEST(bit_ctz, (uint16_t)0x08, 3);
  TEST(bit_ctz, (uint16_t)0x0f, 0);
  TEST(bit_ctz, (uint16_t)0xf00, 8);
  TEST(bit_ctz, (uint16_t)0x800, 11);

  TEST(bit_ctz32, 0x80, 7);
  TEST(bit_ctz32, 0x08, 3);
  TEST(bit_ctz32, 0x0f, 0);
  TEST(bit_ctz32, 0xf0, 4);
  TEST(bit_ctz32, 0xf00, 8);
  TEST(bit_ctz32, 0x800, 11);
  TEST(bit_ctz32, 0x10000, 16);
  TEST(bit_ctz32, 0x80010000, 16);
  TEST(bit_ctz32, 0x80000000, 31);

  TEST(bit_ctz, (uint32_t)0x80, 7);
  TEST(bit_ctz, (uint32_t)0x08, 3);
  TEST(bit_ctz, (uint32_t)0x0f, 0);
  TEST(bit_ctz, (uint32_t)0xf00, 8);
  TEST(bit_ctz, (uint32_t)0x800, 11);
  TEST(bit_ctz, (uint32_t)0x10000, 16);
  TEST(bit_ctz, (uint32_t)0x80010000, 16);
  TEST(bit_ctz, (uint32_t)0x80000000, 31);

  TEST(bit_ctz64, 0x80, 7);
  TEST(bit_ctz64, 0x08, 3);
  TEST(bit_ctz64, 0x0f, 0);
  TEST(bit_ctz64, 0xf0, 4);
  TEST(bit_ctz64, 0xf00, 8);
  TEST(bit_ctz64, 0x800, 11);
  TEST(bit_ctz64, 0x10000, 16);
  TEST(bit_ctz64, 0x80000000, 31);
  TEST(bit_ctz64, 0x100000000ULL, 32);
  TEST(bit_ctz64, 0x8000000100000000ULL, 32);
  TEST(bit_ctz64, 0x8000000000000000ULL, 63);

  TEST(bit_ctz, (uint64_t)0x80, 7);
  TEST(bit_ctz, (uint64_t)0x08, 3);
  TEST(bit_ctz, (uint64_t)0x0f, 0);
  TEST(bit_ctz, (uint64_t)0xf00, 8);
  TEST(bit_ctz, (uint64_t)0x800, 11);
  TEST(bit_ctz, (uint64_t)0x10000, 16);
  TEST(bit_ctz, (uint64_t)0x100000000ULL, 32);
  TEST(bit_ctz, (uint64_t)0x8000000100000000ULL, 32);
  TEST(bit_ctz, (uint64_t)0x8000000000000000ULL, 63);

  TEST(bit_ffs8, 0, 0);
  TEST(bit_ffs8, 0x80, 7+1);
  TEST(bit_ffs8, 0x08, 3+1);
  TEST(bit_ffs8, 0x0f, 0+1);
  TEST(bit_ffs8, 0xf0, 4+1);

  TEST(bit_ffs, (uint8_t)0, 0);
  TEST(bit_ffs, (uint8_t)0x80, 7+1);
  TEST(bit_ffs, (uint8_t)0x08, 3+1);
  TEST(bit_ffs, (uint8_t)0x0f, 0+1);
  TEST(bit_ffs, (uint8_t)0xf0, 4+1);

  TEST(bit_ffs16, 0, 0);
  TEST(bit_ffs16, 0x80, 7+1);
  TEST(bit_ffs16, 0x08, 3+1);
  TEST(bit_ffs16, 0x0f, 0+1);
  TEST(bit_ffs16, 0xf0, 4+1);
  TEST(bit_ffs16, 0xf00, 8+1);
  TEST(bit_ffs16, 0x800, 11+1);

  TEST(bit_ffs, (uint16_t)0, 0);
  TEST(bit_ffs, (uint16_t)0x80, 7+1);
  TEST(bit_ffs, (uint16_t)0x08, 3+1);
  TEST(bit_ffs, (uint16_t)0x0f, 0+1);
  TEST(bit_ffs, (uint16_t)0xf00, 8+1);
  TEST(bit_ffs, (uint16_t)0x800, 11+1);

  TEST(bit_ffs32, 0, 0);
  TEST(bit_ffs32, 0x80, 7+1);
  TEST(bit_ffs32, 0x08, 3+1);
  TEST(bit_ffs32, 0x0f, 0+1);
  TEST(bit_ffs32, 0xf0, 4+1);
  TEST(bit_ffs32, 0xf00, 8+1);
  TEST(bit_ffs32, 0x800, 11+1);
  TEST(bit_ffs32, 0x10000, 16+1);
  TEST(bit_ffs32, 0x80010000, 16+1);
  TEST(bit_ffs32, 0x80000000, 31+1);

  TEST(bit_ffs, (uint32_t)0, 0);
  TEST(bit_ffs, (uint32_t)0x80, 7+1);
  TEST(bit_ffs, (uint32_t)0x08, 3+1);
  TEST(bit_ffs, (uint32_t)0x0f, 0+1);
  TEST(bit_ffs, (uint32_t)0xf00, 8+1);
  TEST(bit_ffs, (uint32_t)0x800, 11+1);
  TEST(bit_ffs, (uint32_t)0x10000, 16+1);
  TEST(bit_ffs, (uint32_t)0x80010000, 16+1);
  TEST(bit_ffs, (uint32_t)0x80000000, 31+1);

  TEST(bit_ffs64, 0, 0);
  TEST(bit_ffs64, 0x80, 7+1);
  TEST(bit_ffs64, 0x08, 3+1);
  TEST(bit_ffs64, 0x0f, 0+1);
  TEST(bit_ffs64, 0xf0, 4+1);
  TEST(bit_ffs64, 0xf00, 8+1);
  TEST(bit_ffs64, 0x800, 11+1);
  TEST(bit_ffs64, 0x10000, 16+1);
  TEST(bit_ffs64, 0x80000000, 31+1);
  TEST(bit_ffs64, 0x100000000ULL, 32+1);
  TEST(bit_ffs64, 0x8000000100000000ULL, 32+1);
  TEST(bit_ffs64, 0x8000000000000000ULL, 63+1);

  TEST(bit_ffs, (uint64_t)0, 0);
  TEST(bit_ffs, (uint64_t)0x80, 7+1);
  TEST(bit_ffs, (uint64_t)0x08, 3+1);
  TEST(bit_ffs, (uint64_t)0x0f, 0+1);
  TEST(bit_ffs, (uint64_t)0xf00, 8+1);
  TEST(bit_ffs, (uint64_t)0x800, 11+1);
  TEST(bit_ffs, (uint64_t)0x10000, 16+1);
  TEST(bit_ffs, (uint64_t)0x80000000ULL, 31+1);
  TEST(bit_ffs, (uint64_t)0x100000000ULL, 32+1);
  TEST(bit_ffs, (uint64_t)0x8000000100000000ULL, 32+1);
  TEST(bit_ffs, (uint64_t)0x8000000000000000ULL, 63+1);

  TEST(bit_popc8, 0, 0);
  TEST(bit_popc8, 0x80, 1);
  TEST(bit_popc8, 0x08, 1);
  TEST(bit_popc8, 0x0f, 4);
  TEST(bit_popc8, 0xf0, 4);
  TEST(bit_popc8, 0x81, 2);
  TEST(bit_popc8, 0xff, 8);
  TEST(bit_popc8, 0xf00, 0);

  TEST(bit_popc, (uint8_t)0, 0);
  TEST(bit_popc, (uint8_t)0x80, 1);
  TEST(bit_popc, (uint8_t)0x08, 1);
  TEST(bit_popc, (uint8_t)0x0f, 4);
  TEST(bit_popc, (uint8_t)0xff, 8);
  TEST(bit_popc, (uint8_t)0x81, 2);

  TEST(bit_popc16, 0, 0);
  TEST(bit_popc16, 0x80, 1);
  TEST(bit_popc16, 0x08, 1);
  TEST(bit_popc16, 0x0f, 4);
  TEST(bit_popc16, 0xf0, 4);
  TEST(bit_popc16, 0x81, 2);
  TEST(bit_popc16, 0xf00, 4);
  TEST(bit_popc16, 0x800, 1);
  TEST(bit_popc16, 0x80a0, 3);
  TEST(bit_popc16, 0x10000, 0);

  TEST(bit_popc, (uint16_t)0, 0);
  TEST(bit_popc, (uint16_t)0x80, 1);
  TEST(bit_popc, (uint16_t)0x08, 1);
  TEST(bit_popc, (uint16_t)0x0f, 4);
  TEST(bit_popc, (uint16_t)0x81, 2);
  TEST(bit_popc, (uint16_t)0xf00, 4);
  TEST(bit_popc, (uint16_t)0x800, 1);
  TEST(bit_popc, (uint16_t)0x80a0, 3);

  TEST(bit_popc32, 0, 0);
  TEST(bit_popc32, 0x80, 1);
  TEST(bit_popc32, 0x08, 1);
  TEST(bit_popc32, 0x0f, 4);
  TEST(bit_popc32, 0xf0, 4);
  TEST(bit_popc32, 0x81, 2);
  TEST(bit_popc32, 0xf00, 4);
  TEST(bit_popc32, 0x800, 1);
  TEST(bit_popc32, 0x80a0, 3);
  TEST(bit_popc32, 0x10000, 1);
  TEST(bit_popc32, 0x10505, 5);
  TEST(bit_popc32, 0x80000000, 1);
  TEST(bit_popc32, 0x8050c107, 9);
  TEST(bit_popc32, 0x100000000ULL, 0);

  TEST(bit_popc, (uint32_t)0, 0);
  TEST(bit_popc, (uint32_t)0x80, 1);
  TEST(bit_popc, (uint32_t)0x08, 1);
  TEST(bit_popc, (uint32_t)0x0f, 4);
  TEST(bit_popc, (uint32_t)0x81, 2);
  TEST(bit_popc, (uint32_t)0xf00, 4);
  TEST(bit_popc, (uint32_t)0x800, 1);
  TEST(bit_popc, (uint32_t)0x80a0, 3);
  TEST(bit_popc, (uint32_t)0x10000, 1);
  TEST(bit_popc, (uint32_t)0x10505, 5);
  TEST(bit_popc, (uint32_t)0x80000000, 1);
  TEST(bit_popc, (uint32_t)0x8050c107, 9);

  TEST(bit_popc64, 0, 0);
  TEST(bit_popc64, 0x80, 1);
  TEST(bit_popc64, 0x08, 1);
  TEST(bit_popc64, 0x0f, 4);
  TEST(bit_popc64, 0xf0, 4);
  TEST(bit_popc64, 0x81, 2);
  TEST(bit_popc64, 0xf00, 4);
  TEST(bit_popc64, 0x800, 1);
  TEST(bit_popc64, 0x80a0, 3);
  TEST(bit_popc64, 0x10000, 1);
  TEST(bit_popc64, 0x10505, 5);
  TEST(bit_popc64, 0x80000000, 1);
  TEST(bit_popc64, 0x8050c107, 9);
  TEST(bit_popc64, 0x100000000ULL, 1);
  TEST(bit_popc64, 0x8000000100000000ULL, 2);
  TEST(bit_popc64, 0x8000000000000000ULL, 1);
  TEST(bit_popc64, 0x0050c107010f8061ULL, 17);

  TEST(bit_popc, (uint64_t)0, 0);
  TEST(bit_popc, (uint64_t)0x80, 1);
  TEST(bit_popc, (uint64_t)0x08, 1);
  TEST(bit_popc, (uint64_t)0x0f, 4);
  TEST(bit_popc, (uint64_t)0x81, 2);
  TEST(bit_popc, (uint64_t)0xf00, 4);
  TEST(bit_popc, (uint64_t)0x800, 1);
  TEST(bit_popc, (uint64_t)0x80a0, 3);
  TEST(bit_popc, (uint64_t)0x10000, 1);
  TEST(bit_popc, (uint64_t)0x10505, 5);
  TEST(bit_popc, (uint64_t)0x80000000, 1);
  TEST(bit_popc, (uint64_t)0x8050c107, 9);
  TEST(bit_popc, (uint64_t)0x100000000ULL, 1);
  TEST(bit_popc, (uint64_t)0x8000000100000000ULL, 2);
  TEST(bit_popc, (uint64_t)0x8000000000000000ULL, 1);
  TEST(bit_popc, (uint64_t)0x0050c107010f8061ULL, 17);

  printk("++SUCCESS++\n");

  power_shutdown();
  power_reboot();
}

