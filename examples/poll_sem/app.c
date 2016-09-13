
#include <mutek/semaphore.h>
#include <mutek/startup.h>
#include <mutek/printk.h>
#include <mutek/thread.h>

#include <device/class/timer.h>

static CONTEXT_ENTRY(thread_entry)
{
  struct semaphore_s sem;
  semaphore_init(&sem, 0);

  struct semaphore_poll_s sem_poll[2];
  semaphore_poll_init(sem_poll, 2, &sem);

  /* get a timer device */
  struct device_timer_s timer_dev;
  ensure(!device_get_accessor_by_path(&timer_dev.base, NULL, "timer*",
                                      DRIVER_CLASS_TIMER));

  /* asynchronous event source 0 */
  struct dev_timer_rq_s timer_rq0;
  dev_request_poll_init(&timer_rq0.rq, &sem_poll[0]);

  /*  -> timer events after 1s */
  ensure(dev_timer_init_sec(&timer_dev, &timer_rq0.delay, NULL, 1, 1) == 0);
  timer_rq0.deadline = 0;
  timer_rq0.rev = 0;

  /* asynchronous event source 1 */
  struct dev_timer_rq_s timer_rq1;
  dev_request_poll_init(&timer_rq1.rq, &sem_poll[1]);

  /*  -> timer events after 700ms */
  ensure(dev_timer_init_sec(&timer_dev, &timer_rq1.delay, NULL, 700, 1000) == 0);
  timer_rq1.deadline = 0;
  timer_rq1.rev = 0;

  /* mask of operations to start */
  semaphore_value_t m = 3;

  while (1)
    {
      /* (re)start some operations */
      if (m & 1)
        DEVICE_OP(&timer_dev, request, &timer_rq0);

      if (m & 2)
        DEVICE_OP(&timer_dev, request, &timer_rq1);

#if 1
      /* wait for at least one operation to terminate */
      m = semaphore_take_any(&sem);
#else
      /* wait for *all* operations to terminate */
      semaphore_take(&sem, m);
#endif

      if (m & 1)
        printk("timeout 0\n");

      if (m & 2)
        printk("timeout 1\n");
    }
}

void app_start()
{
  thread_create(thread_entry, NULL, NULL);
}

