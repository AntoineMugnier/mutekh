
/*
   Additional topics covered by this example:
     - Use of a per device context structure
     - Allocation and release of the context by the driver

   This example implements a char driver which simply counts how many
   bytes are written to the virtual device. When reading from the
   device, the value of the counter is then reported in human readable
   ascii format.
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>

DRIVER_PV(struct mydrv_context_s
{
  /* Our per device char counter. */
  uint32_t counter;
});

#define mydrv_use dev_use_generic

static DEV_CHAR_REQUEST(mydrv_counter_request)
{
  /* retrieve pointer to device */
  struct device_s *dev = accessor->dev;

  /* retrieve pointer to our private data allocated in mydrv_init */
  struct mydrv_context_s *pv = dev->drv_pv;

  rq->error = 0;

  /* This enters a crtical section up to the end of the scope. This
     ensures that a single processor will read/modify/write the
     counter at once. */
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_NONBLOCK:
    case DEV_CHAR_WRITE_NONBLOCK_FLUSH:

      /* simply keep track of amount of data written,
         discard actual data content */
      pv->counter += rq->size;

      /* update request as requested by the char class API */
      rq->data += rq->size;
      rq->size = 0;
      break;

    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_NONBLOCK: {
      /* write text counter in the request provided buffer */
      size_t size = snprintf((char *)rq->data, rq->size,
                             "%u", pv->counter);

      /* update request as requested by the char class API */
      rq->data += size;
      rq->size -= size;
      break;
    }

    case DEV_CHAR_READ:
      /* unable to fill a whole buffer with our counter text */
    default:
      rq->error = -ENOTSUP;
      break;
    }

  dev_char_rq_done(rq);
}

static DEV_CHAR_CANCEL(mydrv_counter_cancel)
{
  return -EBUSY;
}

static DEV_INIT(mydrv_init)
{
  struct mydrv_context_s *pv;

  /* allocate our per device private data used
     to store the counter. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  /* keep the pointer attached to the device */
  dev->drv_pv = pv;

  /* set initial counter value */
  pv->counter = 0;

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  /* retrieve pointer to private data */
  struct mydrv_context_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(mydrv2_drv, 0, "Simple char counter driver example", mydrv,
               DRIVER_CHAR_METHODS(mydrv_counter)
               );

DRIVER_REGISTER(mydrv2_drv);

