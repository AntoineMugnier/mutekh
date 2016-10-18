
#include <mutek/printk.h>
#include <hexo/power.h>
#include <hexo/atomic.h>
#include <assert.h>

void app_start()
{
  atomic_t a;

  atomic_set(&a, 0x42);

  ensure(atomic_get(&a) == 0x42);

  ensure(atomic_add(&a, 2) == 0x42);
  ensure(atomic_get(&a) == 0x44);

  ensure(atomic_or(&a, 2) == 0x44);
  ensure(atomic_get(&a) == 0x46);

  ensure(atomic_xor(&a, 3) == 0x46);
  ensure(atomic_get(&a) == 0x45);

  ensure(atomic_and(&a, 3) == 0x45);
  ensure(atomic_get(&a) == 0x1);

  ensure(atomic_swap(&a, 0x42) == 0x1);
  ensure(atomic_get(&a) == 0x42);

  ensure(atomic_inc(&a));
  ensure(atomic_get(&a) == 0x43);

  ensure(atomic_dec(&a));
  ensure(atomic_get(&a) == 0x42);

  atomic_set(&a, 1);

  ensure(!atomic_dec(&a));
  ensure(atomic_get(&a) == 0);

  atomic_set(&a, -1);

  ensure(!atomic_inc(&a));
  ensure(atomic_get(&a) == 0);

  ensure(!atomic_bit_testset(&a, 3));
  ensure(atomic_get(&a) == 0x08);

  ensure(atomic_bit_testset(&a, 3));
  ensure(atomic_get(&a) == 0x08);
  ensure(atomic_bit_test(&a, 3));

  ensure(atomic_bit_testclr(&a, 3));
  ensure(atomic_get(&a) == 0);

  ensure(!atomic_bit_testclr(&a, 3));
  ensure(atomic_get(&a) == 0);
  ensure(!atomic_bit_test(&a, 3));

  atomic_bit_set(&a, 2);
  ensure(atomic_bit_test(&a, 2));

  atomic_bit_clr(&a, 2);
  ensure(!atomic_bit_test(&a, 2));

  ensure(atomic_compare_and_swap(&a, 0, 4));
  ensure(!atomic_compare_and_swap(&a, 0, 4));
  ensure(atomic_compare_and_swap(&a, 4, 42));
  ensure(atomic_compare_and_swap(&a, 42, 4));

  printk("++SUCCESS++\n");

  power_shutdown();
  power_reboot();
}

