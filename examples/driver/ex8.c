/*
   Additional topics covered by this example:
     - Use a request queue and process request asynchronously

   This is an improved version of the example 3 UART driver. This
   driver supports read operation which requires asynchronous
   handling. This use interrupts (example 6) and a device request
   queue.

   For this driver initialization to succeed, any static instance of
   the device must specify the memory address range of the UART
   and the irq line used:

   DEV_DECLARE_STATIC(my_dev8, "mydev8", 0, mydrv8_drv,
                      DEV_STATIC_RES_MEM(0xd0200000, 0xd0200010),
                      DEV_STATIC_RES_DEV_ICU("/icu"),
                      DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                      );
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/error.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/request.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>

/* Hardware registers */
enum {
  REG_WRITE  = 0,
  REG_STATUS = 4,
  REG_READ   = 8
};

DRIVER_PV(struct mydrv_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_eps;

  /* The queue of pending read requests */
  dev_request_queue_root_t read_q;
});


/* This function read bytes from the UART and fill buffer of requests
   in queue order.

   It returns when there is no more data available from the UART or
   when there is no more request to serve. True is returned in the
   later case. */
static bool_t mydrv_do_read(struct mydrv_context_s *pv)
{
  while (1)
    {
      /* get the next request from our queue */
      struct dev_char_rq_s *rq;
      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q));

      if (rq == NULL)
        return 1;               /* no more request to serve */

      if (rq->size == 0) /* terminate rq if no buffer space is left */
        {
          /* remove the request from the queue */
          dev_request_queue_pop(&pv->read_q);

          /* signal the request owner that we did the job eventually */
          kroutine_exec(&rq->base.kr);
        }

      else if (/* test if at least one byte is available from the UART. */
               endian_le32(cpu_mem_read_32(pv->addr + REG_STATUS)) & 1)
        {
          /* transfer one byte from the UART to the buffer request */
          *rq->data++ = endian_le32(cpu_mem_read_32(pv->addr + REG_READ));
          rq->size--;
        }

      else
        {
          /* will get more data on next call to fill
             the whole request buffer hopefully */
          return 0;
        }
    }
}

static DEV_CHAR_REQUEST(mydrv_uart_request)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  rq->error = 0;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ:
      /* Push the read request at the end of our queue. */
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));

      /* Try to process any requests present on the queue. */
      mydrv_do_read(pv);

      /* Do not execute kroutine_exec before the function return
         because the request may not be processed yet. */
      return;

    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_NONBLOCK:
    case DEV_CHAR_WRITE_NONBLOCK_FLUSH:
      /* No queue is used for write requests in this simple example. */
      while (rq->size != 0)
        {
          cpu_mem_write_32(pv->addr + REG_WRITE, endian_le32(*rq->data++));
          rq->size--;
        }

      kroutine_exec(&rq->base.kr);
      return;

    case DEV_CHAR_WRITE_POLL:
      /* we are always ready to write */
      kroutine_exec(&rq->base.kr);
      return;

    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_POLL:
    case DEV_CHAR_READ_NONBLOCK:
    case DEV_CHAR_DISCARD:
      /* for clarity, these request types are supported in the next example */

    default:
      rq->error = -ENOTSUP;
      kroutine_exec(&rq->base.kr);
      return;
    }
}

static DEV_CHAR_CANCEL(mydrv_uart_cancel)
{
  /* cancellation is supported in the next example */
  return -ENOTSUP;
}

static DEV_IRQ_SRC_PROCESS(mydrv_uart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  /* The UART raised an irq, we should have
     data available to serve queued read requests. */
  if (mydrv_do_read(pv))
    {
      /* In case there is no request in the queue, we still need to
         read some data from the hardware so that the irq line becomes
         inactive eventually. */
      cpu_mem_read_32(pv->addr + REG_READ);
    }

  lock_release(&dev->lock);
}

#define mydrv_use dev_use_generic

static DEV_INIT(mydrv_init)
{
  struct mydrv_context_s *pv;

  /* get reg base address */
  uintptr_t addr;
  if (device_res_get_mem(dev, 0, &addr, NULL))
    return -EINVAL;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  pv->addr = addr;

  /* initializes our queue of read requests */
  dev_request_queue_init(&pv->read_q);

  /* setup irq line */
  device_irq_source_init(dev, &pv->irq_eps, 1, mydrv_uart_irq);
  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    {
      mem_free(pv);
      return -EINVAL;
    }

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  struct mydrv_context_s *pv = dev->drv_pv;

  /* does not allow device cleanup when there are pending requests. */
  if (!dev_request_queue_isempty(&pv->read_q))
    return -EBUSY;

  /* cleanup our queue of read requests */
  dev_request_queue_destroy(&pv->read_q);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(mydrv8_drv, 0, "Simple UART driver example with irqs", mydrv,
               DRIVER_CHAR_METHODS(mydrv_uart)
               );

DRIVER_REGISTER(mydrv8_drv);
