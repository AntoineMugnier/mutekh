
/*
  Additional topics covered by this example:
     - Declaration and implementation of a single device driver API
     - Simple char device request handling

  This example implements a /dev/null kind of char driver. Read
  operations always return zeros and write operations simply discard
  data.
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
  /* no device state to track in this simple driver. */
});

/* This function is required by the DRIVER_CHAR_METHODS(mydrv_null)
   class declaration below. It requests the driver to start an
   asynchronous read or write operation as explained in the char
   device class API documentation. */
static DEV_CHAR_REQUEST(mydrv_null_request)
{
  /* no error is the common case. */
  rq->error = 0;

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_NONBLOCK:
      /* fill provided buffer with zeros */
      memset(rq->data, 0, rq->size);

      /* update request as requested by the char class API */
      rq->data += rq->size;
      rq->size = 0;
    case DEV_CHAR_DISCARD:
      break;

    case DEV_CHAR_READ_POLL:
      /* this device is always ready to source data. */
      break;

    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_NONBLOCK:
    case DEV_CHAR_WRITE_NONBLOCK_FLUSH:
      /* do nothing with written data */

      /* update request as requested by the char class API */
      rq->data += rq->size;
      rq->size = 0;
      break;

    case DEV_CHAR_WRITE_POLL:
      /* this device is always ready to sink data. */
      break;

    default:
      /* Other operations are not supported. */
      rq->error = -ENOTSUP;
      break;
    }

  /* Signal the caller that we are done with its request. In more
     complex drivers this is often deferred because we rely on an
     hardware interrupt or on an asynchronous operation we started on
     an other device. */
  kroutine_exec(&rq->base.kr);
}

/* This function is required by the DRIVER_CHAR_METHODS(mydrv_null)
   class declaration below. It requests the driver to cancel a running
   operation as explained in the doc. */
static DEV_CHAR_CANCEL(mydrv_null_cancel)
{
  /* Because any request is ended before the mydrv_null_request
     function actually returns in our simple driver, the last request
     has already terminated when this function is called. So always
     returning a busy status is the right thing to do in this simple
     case. */
  return -EBUSY;
}

#define mydrv_use dev_use_generic

static DEV_INIT(mydrv_init)
{
  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  return 0;
}

DRIVER_DECLARE(mydrv1_drv, 0, "Char /dev/null driver", mydrv,

               /* implement a single class: the char device class */

               DRIVER_CHAR_METHODS(/* prefix for methods of the class. */
                                   mydrv_null
                                   )
               );

DRIVER_REGISTER(mydrv1_drv);

