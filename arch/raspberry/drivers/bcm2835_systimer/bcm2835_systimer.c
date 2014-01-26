/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

/* systimer registers */
#define BCM2835_SYSTIMER_STATUS  0x00
#define BCM2835_SYSTIMER_CLOW    0x04
#define BCM2835_SYSTIMER_CHIGH   0x08
#define BCM2835_SYSTIMER_CMP(n)  (0x0c + 4 * (n))

/* channels 0 and 2 are used by the GPU, 1 and 3 are free */
#define BCM2835_SYSTIMER_CMP_CHANNEL 1

struct bcm2835_systimer_private_s
{
  /* Timer address */
  uintptr_t addr;
#ifdef CONFIG_DEVICE_IRQ
  /* Interrupt end-points */
  struct dev_irq_ep_s irq_eps[4];
  /* Request queue */
  dev_timer_queue_root_t queue;
  uint32_t skew;
#endif
};

static uint64_t get_timer_value(struct bcm2835_systimer_private_s *pv)
{
  uint32_t lo, hi;

  hi = cpu_mem_read_32(pv->addr + BCM2835_SYSTIMER_CHIGH);
  lo = cpu_mem_read_32(pv->addr + BCM2835_SYSTIMER_CLOW);

  if (hi != cpu_mem_read_32(pv->addr + BCM2835_SYSTIMER_CHIGH))
    lo = cpu_mem_read_32(pv->addr + BCM2835_SYSTIMER_CLOW);

  return ((uint64_t)endian_le32(hi) << 32) | endian_le32(lo);
}

#ifdef CONFIG_DEVICE_IRQ

static void set_timer_compare(struct bcm2835_systimer_private_s *pv, dev_timer_value_t deadline)
{
  uint32_t skew = pv->skew;

  while (1)
    {
      uint64_t now = get_timer_value(pv);

      if (deadline > now)
        {
          cpu_mem_write_32(pv->addr + BCM2835_SYSTIMER_STATUS, endian_le32(1 << BCM2835_SYSTIMER_CMP_CHANNEL));
          cpu_mem_write_32(pv->addr + BCM2835_SYSTIMER_CMP(BCM2835_SYSTIMER_CMP_CHANNEL), endian_le32(deadline));

          if (deadline > get_timer_value(pv))
            break;
        }

      deadline = now + skew;
      pv->skew = skew;
      skew *= 2;
    }
}

static DEV_IRQ_EP_PROCESS(bcm2835_systimer_irq)
{
  struct device_s *dev = ep->dev;
  struct bcm2835_systimer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t status = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_SYSTIMER_STATUS))
        & (1 << BCM2835_SYSTIMER_CMP_CHANNEL);

      if (!status)
        break;

      cpu_mem_write_32(pv->addr + BCM2835_SYSTIMER_STATUS, endian_le32(status));

      struct dev_timer_rq_s *rq;

      while ((rq = dev_timer_queue_head(&pv->queue)))
        {
          if (rq->deadline > get_timer_value(pv))
            break;

          dev_timer_queue_pop(&pv->queue);
          rq->drvdata = 0;

          struct dev_timer_rq_s *next = dev_timer_queue_head(&pv->queue);
          if (next != NULL && next->deadline > get_timer_value(pv))
            cpu_mem_write_32(pv->addr + BCM2835_SYSTIMER_CMP(BCM2835_SYSTIMER_CMP_CHANNEL),
                             endian_le32(next->deadline));

          lock_release(&dev->lock);
          kroutine_exec(&rq->kr, 0);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}
#endif

static DEVTIMER_CANCEL(bcm2835_systimer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct bcm2835_systimer_private_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  assert(rq->tdev == tdev);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->drvdata == pv)
    {
      struct dev_timer_rq_s *rq0 = dev_timer_queue_head(&pv->queue);
     
      dev_timer_queue_remove(&pv->queue, rq);
      rq->drvdata = NULL;
     
      if (rq == rq0)       /* removed first request ? */
        {
          rq0 = dev_timer_queue_head(&pv->queue);

          if (rq0 != NULL)
            set_timer_compare(pv, rq0->deadline);
        }

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEVTIMER_REQUEST(bcm2835_systimer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct bcm2835_systimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  rq->tdev = tdev;

  LOCK_SPIN_IRQ(&dev->lock);

  uint64_t value = get_timer_value(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value)
    err = ETIMEDOUT;
  else
    {
      dev_timer_queue_insert_ascend(&pv->queue, rq);
      rq->drvdata = pv;

      if (dev_timer_queue_head(&pv->queue) == rq)
        set_timer_compare(pv, rq->deadline);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEVTIMER_START_STOP(bcm2835_systimer_state_start_stop)
{
  return 0;
}

static DEVTIMER_GET_VALUE(bcm2835_systimer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct bcm2835_systimer_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVTIMER_RESOLUTION(bcm2835_systimer_resolution)
{
  error_t err = 0;

  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = 1;
    }

  if (max)
    *max = 0xffffffffffffffffULL;

  return err;
}

const struct driver_timer_s  bcm2835_systimer_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = bcm2835_systimer_request,
  .f_cancel       = bcm2835_systimer_cancel,
  .f_start_stop   = bcm2835_systimer_state_start_stop,
  .f_get_value    = bcm2835_systimer_get_value,
  .f_resolution   = bcm2835_systimer_resolution,
};

/************************************************************************/

static DEV_INIT(bcm2835_systimer_init);
static DEV_CLEANUP(bcm2835_systimer_cleanup);

const struct driver_s  bcm2835_systimer_drv =
{
  .desc           = "EFM32 Timer",
  .f_init         = bcm2835_systimer_init,
  .f_cleanup      = bcm2835_systimer_cleanup,

  .classes        = {
    &bcm2835_systimer_timer_drv,
    0
  }
};

static DEV_INIT(bcm2835_systimer_init)
{
  struct bcm2835_systimer_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct bcm2835_systimer_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, pv->irq_eps, 4,
                         bcm2835_systimer_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_eps, 4, 1 << BCM2835_SYSTIMER_CMP_CHANNEL))
    goto err_mem;

  dev_timer_queue_init(&pv->queue);
  pv->skew = 1;
#endif

  dev->drv = &bcm2835_systimer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_DEVICE_IRQ
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(bcm2835_systimer_cleanup)
{
  struct bcm2835_systimer_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  dev_timer_queue_destroy(&pv->queue);

  device_irq_source_unlink(dev, pv->irq_eps, 4);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(bcm2835_systimer_drv);

