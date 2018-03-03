/*
   Additional topics covered by this example:
     - Handle more char API request types
     - Implement request cancellation

   This is an improved version of the previous example. This
   driver allow handling of all kind of read operation.
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

  dev_request_queue_root_t read_q;
});


/* This function return true if at least
   one byte can be read from the UART. */
static inline bool_t mydrv_rx_ready(struct mydrv_context_s *pv)
{
  return endian_le32(cpu_mem_read_32(pv->addr + REG_STATUS)) & 1;
}

/* This function copies one byte from the UART
   to the buffer of the specified request. */
static inline void mydrv_read_byte(struct mydrv_context_s *pv,
                                   struct dev_char_rq_s *rq)
{
  *rq->data++ = endian_le32(cpu_mem_read_32(pv->addr + REG_READ));
  rq->size--;
}

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
        return 1;

      if (rq->size == 0)
        goto end_rq;         /* no buffer space left in this request */

      /* different types of read request require different
         behavior here, as specified in the char class API. */
      switch (rq->type)
        {
        case DEV_CHAR_READ:
          if (!mydrv_rx_ready(pv))
            return 0;        /* will get more data on next call to
                                fill the whole buffer */

          mydrv_read_byte(pv, rq);

          continue;          /* see if we are able to get more */

        case DEV_CHAR_DISCARD:
          if (!mydrv_rx_ready(pv))
            return 0;        /* will get more data on next call to
                                serve the whole request */

          cpu_mem_read_32(pv->addr + REG_READ); /* discard one byte */
          rq->size--;

          continue;          /* see if we are able to get more */

        case DEV_CHAR_READ_PARTIAL:
          if (!mydrv_rx_ready(pv))
            return 0;        /* will get at least one byte of data on
                                next call */

          mydrv_read_byte(pv, rq);

          if (!mydrv_rx_ready(pv))
            goto end_rq;     /* we were able to read at least 1 bytes,
                                we are done */

          continue;          /* get one more byte */

        case DEV_CHAR_READ_NONBLOCK:
          if (!mydrv_rx_ready(pv))
            goto end_rq;     /* we are done even if we are not able to
                                get any data at all */

          mydrv_read_byte(pv, rq);

          continue;          /* try to get more */

        case DEV_CHAR_READ_POLL:
          if (!mydrv_rx_ready(pv))
            return 0;        /* wait for data available on next call */

          goto end_rq;       /* data is available, we are done */

        default:
          UNREACHABLE();
        }

    end_rq:
      /* remove the request at the queue head */
      dev_request_queue_pop(&pv->read_q);

      /* remove our marking so that any call to
         mydrv_uart_cancel will fail properly. */
      rq->base.drvdata = NULL;

      /* signal the request owner that we did the job eventually */
      kroutine_exec(&rq->base.kr);

      /* see if we can serve more requests present on the queue with
         more data from the UART */
      continue;
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
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_POLL:
    case DEV_CHAR_READ_NONBLOCK:
    case DEV_CHAR_DISCARD:
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));

      /* We use a driver reserved field in the request to mark the
         request as currently processed by the device. */
      rq->base.drvdata = dev;

      mydrv_do_read(pv);
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
      kroutine_exec(&rq->base.kr);
      return;

    default:
      rq->error = -ENOTSUP;
      kroutine_exec(&rq->base.kr);
      return;
    }
}

static DEV_CHAR_CANCEL(mydrv_uart_cancel)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_POLL:
    case DEV_CHAR_READ_NONBLOCK:
    case DEV_CHAR_DISCARD:
      /* Because the execution of the kroutine used to signal an end of
         request may be deferred, the owner of the request may well call
         the cancel function on a request which as already terminated.

         We use our marking of the request to handle this case properly as
         requested by the char API. */
      if (rq->base.drvdata != dev)
        return -EBUSY;

      /* We have to forget the request and remove our marking. */
      dev_request_queue_remove(&pv->read_q, dev_char_rq_s_base(rq));
      rq->base.drvdata = NULL;

      return 0;

    default:
      /* Any other type of request has been served synchronously in
         the mydrv_uart_request, so it already terminated. */
      return -EBUSY;
    }
}

static DEV_IRQ_SRC_PROCESS(mydrv_uart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  /* try to handle queued requests
     and ack the irq in any case. */
  if (mydrv_do_read(pv))
    cpu_mem_read_32(pv->addr + REG_READ);

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

  if (!dev_request_queue_isempty(&pv->read_q))
    return -EBUSY;

  dev_request_queue_destroy(&pv->read_q);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(mydrv9_drv, 0, "Simple UART driver example with irqs", mydrv,
               DRIVER_CHAR_METHODS(mydrv_uart)
               );

DRIVER_REGISTER(mydrv9_drv);
