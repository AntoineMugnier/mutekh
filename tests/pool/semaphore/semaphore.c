
#include <mutek/printk.h>
#include <mutek/thread.h>
#include <mutek/semaphore.h>
#include <mutek/startup.h>

#include <hexo/power.h>

static struct semaphore_s sem;
static struct semaphore_s barrier2;
static struct semaphore_s barrier3;
static uint_fast8_t state = 0;

static void test_yield()
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_context_switch();
  CPU_INTERRUPT_RESTORESTATE;
}

static void test2_thread1()
{
  semaphore_value_t x;

  semaphore_take(&sem, 1);      /* 1: count == 1 */
  assert(state == 1);

  semaphore_barrier(&barrier2, 1);

  state = 2;
  semaphore_give(&sem, 1);      /* 3: count == 1 */

  state = 3;
  semaphore_give(&sem, 2);      /* 4: count == 3 */

  semaphore_take(&sem, 1);      /* 5: count == 2 or 0 */
  assert(state == 3);

  semaphore_barrier(&barrier2, 1);

  x = semaphore_take_any(&sem);     /* 7: count == 0 */
  assert(x == 1);
  assert(state == 4);

  semaphore_barrier(&barrier2, 1);

  state = 5;
  semaphore_give(&sem, 3);      /* 8: count == 3 */

  semaphore_barrier(&barrier2, 1);

  semaphore_take(&sem, 1);      /* 1: count == 1 */
  assert(state == 6);

  semaphore_barrier(&barrier2, 1);

  state = 7;
  semaphore_give_any(&sem, 1);      /* 3: count == 1 */

  state = 8;
  semaphore_give_any(&sem, 2);      /* 4: count == 3 */

  semaphore_take(&sem, 1);      /* 5: count == 2 or 0 */
  assert(state == 8);

  semaphore_barrier(&barrier2, 1);

  x = semaphore_take_any(&sem);     /* 7: count == 0 */
  assert(x == 5);
  assert(state == 9);

  semaphore_barrier(&barrier2, 1);

  state = 10;
  semaphore_give_any(&sem, 3);      /* 8: count == 3 */
}

static void test2_thread2()
{
  semaphore_value_t x;

  assert(state == 0);
  state = 1;
  semaphore_give(&sem, 1);      /* 2: count == 0 */

  semaphore_barrier(&barrier2, 1);

  semaphore_take(&sem, 2);      /* 5: count == 1 or 0 */
  assert(state == 3);

  semaphore_barrier(&barrier2, 1);

  state = 4;
  semaphore_give(&sem, 1);      /* 6: count == 1 */

  semaphore_barrier(&barrier2, 1);

  semaphore_take(&sem, 1);      /* 9: count == 2 */
  x = semaphore_take_any(&sem);     /* 10: count == 0 */
  assert(x == 2);
  assert(state == 5);

  semaphore_barrier(&barrier2, 1);

  state = 6;
  semaphore_give_any(&sem, 1);      /* 2: count == 0 */

  semaphore_barrier(&barrier2, 1);

  semaphore_take(&sem, 2);      /* 5: count == 1 or 0 */
  assert(state == 8);

  semaphore_barrier(&barrier2, 1);

  state = 9;
  semaphore_give_any(&sem, 5);      /* 6: count == 1 */

  semaphore_barrier(&barrier2, 1);

  semaphore_take(&sem, 1);      /* 9: count == 2 */
  x = semaphore_take_any(&sem);     /* 10: count == 0 */
  assert(x == 2);
  assert(state == 10);
}

static void test3_thread1()
{
  semaphore_take(&sem, 1);      /* 1: count == 1 or 0 */
  assert(state == 11);

  semaphore_barrier(&barrier3, 1); /******************/

  semaphore_take(&sem, 3);      /* 3 */
  assert(state == 13);

  semaphore_barrier(&barrier3, 1); /******************/

  semaphore_take(&sem, 3);      /* 7 */
  assert(state == 15);

  semaphore_barrier(&barrier2, 1);
}

static void test3_thread2()
{
  semaphore_take(&sem, 1);      /* 2: count == 1 or 0 */
  assert(state == 11);

  semaphore_barrier(&barrier3, 1); /******************/

  /* ensure thread 1 waits first */
  while (semaphore_value(&sem) >= 0)
    test_yield();

  semaphore_take(&sem, 1);      /* 4 */
  assert(state == 12);

  semaphore_barrier(&barrier2, 1);

  semaphore_barrier(&barrier3, 1); /******************/

  /* ensure thread 1 waits first */
  while (semaphore_value(&sem) >= 0)
    test_yield();

  semaphore_take(&sem, 1);      /* 8 */
  assert(state == 16);
}

static void test3_thread3()
{
  state = 11;
  semaphore_give(&sem, 2);      /* 0: count == 2 */

  semaphore_barrier(&barrier3, 1); /******************/

  /* ensure threads 1 and 2 are blocked */
  while (semaphore_value(&sem) != -4)
    test_yield();

  state = 12;
  semaphore_give_any(&sem, 2);  /* 5 wake thread 1 */

  semaphore_barrier(&barrier2, 1);

  state = 13;
  semaphore_give_any(&sem, 2);  /* 6 wake thread 2 */

  semaphore_barrier(&barrier3, 1); /******************/

  /* ensure threads 1 and 2 are blocked */
  while (semaphore_value(&sem) != -4)
    test_yield();

  state = 14;
  semaphore_give(&sem, 2);  /* 9 do not wake any thread */

  test_yield();

  state = 15;
  semaphore_give(&sem, 1);  /* 10 wake thread 1 */

  semaphore_barrier(&barrier2, 1);

  state = 16;
  semaphore_give(&sem, 1);  /* 11 wake thread 2 */
}

static CONTEXT_ENTRY(thread1_entry)
{
  test2_thread1();

  semaphore_barrier(&barrier3, 1);
  test3_thread1();

  printk("thread 1 done\n");
  semaphore_barrier(&barrier3, 1);

  printk("++SUCCESS++%u++\n", state);

  power_shutdown();
  power_reboot();
}

static CONTEXT_ENTRY(thread2_entry)
{
  test2_thread2();

  semaphore_barrier(&barrier3, 1);
  test3_thread2();

  printk("thread 2 done\n");
  semaphore_barrier(&barrier3, 1);
}

static CONTEXT_ENTRY(thread3_entry)
{
  semaphore_barrier(&barrier3, 1);
  test3_thread3();

  printk("thread 3 done\n");
  semaphore_barrier(&barrier3, 1);
}

void app_start()
{
  semaphore_init(&barrier2, 2);
  semaphore_init(&barrier3, 3);
  semaphore_init(&sem, 0);

  thread_create(thread1_entry, NULL, NULL);
  thread_create(thread2_entry, NULL, NULL);
  thread_create(thread3_entry, NULL, NULL);
}

