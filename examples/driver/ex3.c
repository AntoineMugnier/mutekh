/*
   Additional topics covered by this example:
     - Get instance specific information from device resource entries
     - Drive real hardware
     - Access memory mapped registers

   This example implements a char driver for a simple UART device.

   The hardware UART we drive here has the following
   32 bits little endian memory mapped registers:

       offset 0:  WRITE  : send any byte written to this register
       offset 4:  STATUS : bit 0 is set when at least one byte can be read
       offset 8:  READ   : pop one received byte on read

   The UART has an infinite write fifo. This driver works with the
   SoCLib multi_tty component.

   For this driver initialization to succeed, any static instance of
   the device must specify the memory address range of the UART
   registers by using a resource entry:

   DEV_DECLARE_STATIC(my_dev3, "mydev3", 0, mydrv3_drv,
                      DEV_STATIC_RES_MEM(0xd0200000, 0xd0200010),
                      );
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/error.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>

/* Hardware registers described above */
enum {
  REG_WRITE  = 0,
  REG_STATUS = 4,
  REG_READ   = 8
};

DRIVER_PV(struct mydrv_context_s
{
  /* We keep the base address of the UART memory
     mapped registers in our per device private storage. */
  uintptr_t addr;
});

static DEV_CHAR_REQUEST(mydrv_uart_request)
{
  /* retrieve pointer to our device and private data */
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  uintptr_t reg_addr = pv->addr;

  rq->error = 0;

  /* even if there is no data modified in the driver context, we still
     need to be atomic between device register access. */
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_POLL:
      /* This should be supported in any driver but it requires
         putting the request in a wait queue and waiting for more data
         using an irq. Not implemented in this toy driver. */
    default:
      rq->error = -ENOTSUP;
      break;

    case DEV_CHAR_READ_NONBLOCK:
      while (/* check room left in the request provided buffer. */
             rq->size != 0 &&
             /* test if more bytes are available from the UART. */
             endian_le32(cpu_mem_read_32(reg_addr + REG_STATUS)) & 1)
        {
          /* read a single from the UART into the buffer */
          *rq->data++ = endian_le32(cpu_mem_read_32(reg_addr + REG_READ));
          rq->size--;
        }
      break;

    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_NONBLOCK:
    case DEV_CHAR_WRITE_NONBLOCK_FLUSH:
      /* Send all data from request */
      while (rq->size != 0)
        {
          /* write a single from the buffer into the UART */
          cpu_mem_write_32(reg_addr + REG_WRITE, endian_le32(*rq->data++));
          rq->size--;
        }
      break;

    case DEV_CHAR_WRITE_POLL:
      /* we are always ready to write */
      break;
    }

  kroutine_exec(&rq->base.kr);
}

static DEV_CHAR_CANCEL(mydrv_uart_cancel)
{
  return -EBUSY;
}

#define mydrv_use dev_use_generic

static DEV_INIT(mydrv_init)
{
  struct mydrv_context_s *pv;

  /* The device must have an attached resource entry which specifies
     the address of its memory mapped registers. We query the device
     resources here to get this address. */
  uintptr_t addr;
  if (device_res_get_mem(dev, 0, &addr, NULL))
    return -EINVAL;

  /* allocate our per device private data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  /* keep the pointer attached to the device */
  dev->drv_pv = pv;

  /* store the registers address for further use */
  pv->addr = addr;

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  struct mydrv_context_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(mydrv3_drv, 0, "Simple UART driver example", mydrv,
               DRIVER_CHAR_METHODS(mydrv_uart)
               );

DRIVER_REGISTER(mydrv3_drv);
