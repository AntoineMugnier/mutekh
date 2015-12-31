
#include <mutek/kroutine.h>
#include <mutek/semaphore.h>
#include <mutek/startup.h>
#include <mutek/printk.h>
#include <mutek/thread.h>

#include <device/class/timer.h>

/********** worker thread */

struct semaphore_s      worker_sem;
struct kroutine_queue_s worker_queue;

static CONTEXT_ENTRY(worker_thread)
{
  while (1)
    {
      printk("worker waiting...\n");
      kroutine_queue_wait(&worker_queue);
    }
}

/********** event source */

struct device_timer_s timer_dev;
struct dev_timer_rq_s timer_rq;

static KROUTINE_EXEC(timer_event)
{
  printk("tick!\n");

  /* repost timer event */
  timer_rq.deadline = 0;
  ensure(DEVICE_OP(&timer_dev, request, &timer_rq) == 0);
}

/********** init */

void app_start()
{
  /* init worker thread and kroutine queue */
  semaphore_init(&worker_sem, 0);
  kroutine_queue_init(&worker_queue, &worker_sem);
  thread_create(worker_thread, NULL, NULL);

  /* get timer device */
  ensure(!device_get_accessor_by_path(&timer_dev, NULL, "timer*", DRIVER_CLASS_TIMER));

  /* timer events after 1s */
  ensure(dev_timer_init_sec(&timer_dev, &timer_rq.delay, NULL, 1, 1) == 0);
  timer_rq.deadline = 0;
  timer_rq.rev = 0;

    /* makes the timer_event function execute from the worker thread */
  kroutine_init_queue(&timer_rq.rq.kr, timer_event, &worker_queue);

    /* timer kick off */
  ensure(DEVICE_OP(&timer_dev, request, &timer_rq) == 0);
}

