
#include <mutek/printk.h>
#include <hexo/power.h>
#include <hexo/atomic.h>
#include <assert.h>

void app_start()
{
#define ATOMIC_TEST(type)                       \
  type##_t a;                                   \
                                                \
  type##_set(&a, 0x42);                         \
                                                \
  ensure(type##_get(&a) == 0x42);               \
                                                \
  ensure(type##_add(&a, 2) == 0x42);            \
  ensure(type##_get(&a) == 0x44);               \
                                                \
  ensure(type##_or(&a, 2) == 0x44);             \
  ensure(type##_get(&a) == 0x46);               \
                                                \
  ensure(type##_xor(&a, 3) == 0x46);            \
  ensure(type##_get(&a) == 0x45);               \
                                                \
  ensure(type##_and(&a, 3) == 0x45);            \
  ensure(type##_get(&a) == 0x1);                \
                                                \
  ensure(type##_swap(&a, 0x42) == 0x1);         \
  ensure(type##_get(&a) == 0x42);               \
                                                \
  ensure(type##_inc(&a));                       \
  ensure(type##_get(&a) == 0x43);               \
                                                \
  ensure(type##_dec(&a));                       \
  ensure(type##_get(&a) == 0x42);               \
                                                \
  type##_set(&a, 1);                            \
                                                \
  ensure(!type##_dec(&a));                      \
  ensure(type##_get(&a) == 0);                  \
                                                \
  type##_set(&a, -1);                           \
                                                \
  ensure(!type##_inc(&a));                      \
  ensure(type##_get(&a) == 0);                  \
                                                \
  ensure(!type##_bit_testset(&a, 3));           \
  ensure(type##_get(&a) == 0x08);               \
                                                \
  ensure(type##_bit_testset(&a, 3));            \
  ensure(type##_get(&a) == 0x08);               \
  ensure(type##_bit_test(&a, 3));               \
                                                \
  ensure(type##_bit_testclr(&a, 3));            \
  ensure(type##_get(&a) == 0);                  \
                                                \
  ensure(!type##_bit_testclr(&a, 3));           \
  ensure(type##_get(&a) == 0);                  \
  ensure(!type##_bit_test(&a, 3));              \
                                                \
  type##_bit_set(&a, 2);                        \
  ensure(type##_bit_test(&a, 2));               \
                                                \
  type##_bit_clr(&a, 2);                        \
  ensure(!type##_bit_test(&a, 2));              \
                                                \
  ensure(type##_compare_and_swap(&a, 0, 4));    \
  ensure(!type##_compare_and_swap(&a, 0, 4));   \
  ensure(type##_compare_and_swap(&a, 4, 42));   \
  ensure(type##_compare_and_swap(&a, 42, 4));

  { ATOMIC_TEST(atomic) }
  { ATOMIC_TEST(atomic_fast8) }
  { ATOMIC_TEST(atomic_fast16) }

  atomic_t a = ATOMIC_FAST8_INITIALIZER(42);
  atomic_fast8_t a8 = ATOMIC_FAST8_INITIALIZER(42);
  atomic_fast16_t a16 = ATOMIC_FAST16_INITIALIZER(42);

  printk("++SUCCESS++\n");

  power_shutdown();
  power_reboot();
}

