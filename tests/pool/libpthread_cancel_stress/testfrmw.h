
#include <mutek/printk.h>
#include <hexo/cpu.h>
#include <stdlib.h>
#include <hexo/power.h>
#include <hexo/atomic.h>

cpu_cycle_t test_start;

#define TEST_CYCLES     10000
#define TEST_DELAY_CYCLES 100
#define VERBOSE 2

#define TEST_START                              \
do {                                            \
  test_start = cpu_cycle_count() / 1024;        \
} while (0)

#define TEST_CONTINUE                           \
  (test_start + TEST_CYCLES > cpu_cycle_count() / 1024)

#define TEST_RANDOM_WAIT(divide)                                \
do {                                                            \
  cpu_cycle_wait((rand() * rand() / divide) % TEST_DELAY_CYCLES); \
} while (0)

#define UNRESOLVED(ret, descr)                          \
  do {                                                  \
    printk("++FAILED++:return %i, %s", ret, descr);     \
    power_shutdown();                                   \
    power_reboot();                                     \
  } while (0)

#define FAILED(descr)                           \
  do {                                          \
    printk("++FAILED++:%s", descr);             \
    power_shutdown();                           \
    power_reboot();                             \
  } while (0)

#define PASSED                                  \
  do {                                          \
    printk("++PASSED++");                       \
    power_shutdown();                           \
    power_reboot();                             \
  } while (0)

typedef atomic_t test_cpu_barrier_t;

static inline test_cpu_barrier_init(test_cpu_barrier_t *b)
{
#ifdef CONFIG_ARCH_SMP
  atomic_set(b, 0);
#endif
}

static inline test_cpu_barrier_wait(test_cpu_barrier_t *b)
{
#ifdef CONFIG_ARCH_SMP
  atomic_bit_set(b, cpu_id());

  while (atomic_get(b) != (1 << CONFIG_CPU_MAXCOUNT) - 1)
    order_smp_mem();
#endif
}

#define output(...) printk(__VA_ARGS__)

