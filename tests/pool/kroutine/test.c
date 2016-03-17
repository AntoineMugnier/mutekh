
#include <mutek/printk.h>
#include <hexo/power.h>
#include <mutek/kroutine.h>
#include <hexo/atomic.h>

#define TEST_CTX_CNT 8

atomic_t idle_cnt;

#ifdef CONFIG_HEXO_IRQ
#include <device/class/timer.h>
struct device_timer_s timer_dev;
struct dev_timer_rq_s timer_rq;
#endif

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
#include <mutek/semaphore.h>
#include <mutek/thread.h>
struct semaphore_s sem;

#elif defined(CONFIG_MUTEK_KROUTINE_SCHED)
struct kroutine_s main_loop_kr;

#endif

// #define writek(...)

enum test_policies_e
{
  POLICY_NONE,
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
  POLICY_SCHED,
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
  POLICY_IDLE,
#endif
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  POLICY_TRIGGER,
#endif
  POLICY_count
};

enum test_state_e
{
  STATE_NONE,
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
  STATE_SCHED,
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
  STATE_IDLE,
#endif
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  STATE_TRIGGER_EXEC,
  STATE_TRIGGER_TRIG,
  STATE_TRIGGER_DONE,
#endif
};

struct test_ctx_s
{
  uint32_t count;
  lock_irq_t lock;
  enum test_state_e state;
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
  struct kroutine_s sched_kr;
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
  struct kroutine_s idle_kr;
#endif
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  struct kroutine_s trigger_kr;
#endif
};

struct test_ctx_s contexts[TEST_CTX_CNT];

static void test_ctx(struct test_ctx_s *c, uint32_t *count)
{
  lock_spin_irq(&c->lock);
  if (count)
    *count += c->count;
  switch (c->state)
    {
    case STATE_NONE:
      if (c->count == 0)
        {
          lock_release_irq(&c->lock);
          break;
        }
      switch (rand() % POLICY_count)
        {
        case POLICY_NONE:
          lock_release_irq(&c->lock);
          break;
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
        case POLICY_SCHED:
          c->state = STATE_SCHED;
          lock_release_irq(&c->lock);
          writek("s", 1);
# if !defined(CONFIG_HEXO_CONTEXT_PREEMPT) && defined(CONFIG_MUTEK_CONTEXT_SCHED)
          atomic_inc(&idle_cnt);
# endif
# ifdef CONFIG_HEXO_CONTEXT_IRQEN
          if (rand() % 2)
            {
              CPU_INTERRUPT_SAVESTATE_DISABLE;
              kroutine_exec(&c->sched_kr);
              CPU_INTERRUPT_RESTORESTATE;
            }
          else
# endif
            {
              kroutine_exec(&c->sched_kr);
            }
          break;
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
        case POLICY_IDLE:
          c->state = STATE_IDLE;
          lock_release_irq(&c->lock);
          writek("i", 1);
          atomic_inc(&idle_cnt);
          kroutine_exec(&c->idle_kr);
          break;
#endif
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
        case POLICY_TRIGGER:
          if (rand() & 1)
            {
              c->state = STATE_TRIGGER_EXEC;
              lock_release_irq(&c->lock);
              writek("e", 1);
              kroutine_exec(&c->trigger_kr);
            }
          else
            {
              c->state = STATE_TRIGGER_TRIG;
              lock_release_irq(&c->lock);
              writek("t", 1);
              kroutine_trigger(&c->trigger_kr, KROUTINE_IMMEDIATE /* FIXME */);
            }
          break;
#endif
        default:
          abort();
        }
      break;
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
    case STATE_SCHED:
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
    case STATE_IDLE:
#endif
      lock_release_irq(&c->lock);
      break;
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
    case STATE_TRIGGER_TRIG:
      c->state = STATE_TRIGGER_DONE;
      lock_release_irq(&c->lock);
      writek("e", 1);
      kroutine_exec(&c->trigger_kr);
      break;
    case STATE_TRIGGER_EXEC:
      c->state = STATE_TRIGGER_DONE;
      lock_release_irq(&c->lock);
      writek("t", 1);
      kroutine_trigger(&c->trigger_kr, KROUTINE_IMMEDIATE /* FIXME */);
      break;
    case STATE_TRIGGER_DONE:
      lock_release_irq(&c->lock);
      break;
#endif
    default:
      abort();
    }
}

#ifdef CONFIG_MUTEK_KROUTINE_SCHED
static KROUTINE_EXEC(sched_kr_func)
{
  struct test_ctx_s *c = KROUTINE_CONTAINER(kr, struct test_ctx_s, sched_kr);
  lock_spin_irq(&c->lock);
  assert(c->state == STATE_SCHED);
  c->state = STATE_NONE;
  c->count--;
  lock_release_irq(&c->lock);
  writek("S", 1);
#if !defined(CONFIG_HEXO_CONTEXT_PREEMPT) && defined(CONFIG_MUTEK_CONTEXT_SCHED)
  semaphore_give(&sem, 1);
#endif
  test_ctx(c, NULL);
}
#endif

#ifdef CONFIG_MUTEK_KROUTINE_IDLE
static KROUTINE_EXEC(idle_kr_func)
{
  struct test_ctx_s *c = KROUTINE_CONTAINER(kr, struct test_ctx_s, idle_kr);
  lock_spin_irq(&c->lock);
  assert(c->state == STATE_IDLE);
  c->state = STATE_NONE;
  c->count--;
  lock_release_irq(&c->lock);
  writek("I", 1);
# ifdef CONFIG_MUTEK_CONTEXT_SCHED
  semaphore_give(&sem, 1);
# elif defined(CONFIG_MUTEK_KROUTINE_SCHED)
  kroutine_exec(&main_loop_kr);
# endif
  test_ctx(c, NULL);
}
#endif

#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
static KROUTINE_EXEC(trigger_kr_func)
{
  struct test_ctx_s *c = KROUTINE_CONTAINER(kr, struct test_ctx_s, trigger_kr);
  lock_spin_irq(&c->lock);
  assert(c->state == STATE_TRIGGER_DONE);
  c->state = STATE_NONE;
  c->count--;
  lock_release_irq(&c->lock);
  writek("T", 1);
  test_ctx(c, NULL);
}
#endif

#ifdef CONFIG_HEXO_IRQ
static KROUTINE_EXEC(irq_kr_func)
{
  writek("r", 1);
  uint_fast8_t i;

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
  printk("local %p %p\n", CONTEXT_GET_TLS(), CONTEXT_LOCAL_GET(sched_cur));
#endif

  for (i = 0; i < TEST_CTX_CNT; i++)
    test_ctx(contexts + i, NULL);

  timer_rq.deadline = 0;
  ensure(DEVICE_OP(&timer_dev, request, &timer_rq) == 0);
}
#endif

static void test_init()
{
  uint32_t i;

  atomic_set(&idle_cnt, 0);

  for (i = 0; i < TEST_CTX_CNT; i++)
    {
      struct test_ctx_s *c = contexts + i;
      lock_init_irq(&c->lock);
      c->count = 100;
      c->state = STATE_NONE;
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
      kroutine_init_deferred(&c->sched_kr, &sched_kr_func);
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
      kroutine_init_idle(&c->idle_kr, &idle_kr_func);
#endif
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
      kroutine_init_trigger(&c->trigger_kr, &trigger_kr_func);
#endif
    }

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
  semaphore_init(&sem, 0);
#endif

#ifdef CONFIG_HEXO_IRQ
  ensure(!device_get_accessor_by_path(&timer_dev, NULL, "timer*", DRIVER_CLASS_TIMER));

  ensure(dev_timer_init_sec(&timer_dev, &timer_rq.delay, NULL, 100, 1000) == 0);

  kroutine_init_immediate(&timer_rq.rq.kr, irq_kr_func);
  timer_rq.deadline = 0;
  timer_rq.rev = 0;

  do {
    dev_timer_value_t t;
    error_t e = DEVICE_OP(&timer_dev, request, &timer_rq);
    switch (e)
      {
      case -ETIMEDOUT:
        ensure(DEVICE_OP(&timer_dev, get_value, &t, 0) == 0);
        timer_rq.deadline = t + timer_rq.delay;
        continue;
      case 0:
        break;
      default:
        printk("timer error %i\n", e);
        power_shutdown();
        power_reboot();
      }
  } while (0);
#endif
}

static bool_t test_iter()
{
#if defined(CONFIG_MUTEK_KROUTINE_SCHED) || \
    defined(CONFIG_MUTEK_KROUTINE_IDLE) || \
    defined(CONFIG_MUTEK_KROUTINE_PREEMPT)

  uint32_t i, count = 0;

# if defined(CONFIG_HEXO_CONTEXT_PREEMPT)
  /* spend some time so that preempt can occurs */
  i = rand() % 65536;
  while (i--)
    asm volatile("nop");

  assert(cpu_is_interruptible());
# endif

  for (i = 0; i < TEST_CTX_CNT; i++)
    {
      writek("y", 1);
      test_ctx(contexts + i, &count);
      writek("Y", 1);
    }

  return !count;
#else
  return 1;
#endif
}

static void test_end()
{
  uint32_t i;

  for (i = 0; i < TEST_CTX_CNT; i++)
    assert(contexts[i].state == STATE_NONE);

# if defined(CONFIG_HEXO_CONTEXT_PREEMPT) && defined(CONFIG_MUTEK_KROUTINE_SCHED)
  size_t pcnt = CONTEXT_LOCAL_GET(sched_cur)->context->preempt_cnt;
  assert(pcnt > 0);
  printk("preempt count: %u\n", pcnt);
# endif

  cpu_interrupt_disable();
  printk("++SUCCESS++%u++\n", TEST_CTX_CNT);

  power_shutdown();
  power_reboot();
}

#ifdef CONFIG_MUTEK_CONTEXT_SCHED

static CONTEXT_ENTRY(main_loop_thread)    /* thread based main loop */
{
  printk("main %p %p\n", CONTEXT_GET_TLS(), CONTEXT_LOCAL_GET(sched_cur));

  test_init();

  while (1)
    {
      if (test_iter())
        break;

      /* do we need to force switching to the idle context ? */
      while (atomic_get(&idle_cnt))
        {
          atomic_dec(&idle_cnt);
          semaphore_take(&sem, 1);
        }
    }

  test_end();
}

#elif defined(CONFIG_MUTEK_KROUTINE_SCHED)

static KROUTINE_EXEC(main_loop_func)    /* kroutine based main loop */
{
  if (test_iter())
    {
      test_end();
      return;
    }

  /* do we need to become idle ? */
  if (atomic_get(&idle_cnt))
    {
      atomic_dec(&idle_cnt);
      return;
    }

  kroutine_exec(&main_loop_kr);
}
#endif

void app_start()
{
#ifdef CONFIG_MUTEK_CONTEXT_SCHED
  thread_create(main_loop_thread, NULL, NULL);

#elif defined(CONFIG_MUTEK_KROUTINE_SCHED)
  kroutine_init_sched_switch(&main_loop_kr, &main_loop_func);
  test_init();
  kroutine_exec(&main_loop_kr);

#else
  test_end();
#endif
}

