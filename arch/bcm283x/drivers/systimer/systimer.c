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
#define BCM283X_SYSTIMER_STATUS  0x00
#define BCM283X_SYSTIMER_CLOW    0x04
#define BCM283X_SYSTIMER_CHIGH   0x08
#define BCM283X_SYSTIMER_CMP(n)  (0x0c + 4 * (n))

/* channels 0 and 2 are used by the GPU, 1 and 3 are free */
#define BCM283X_SYSTIMER_CMP_CHANNEL 1

DRIVER_PV(struct bcm283x_systimer_private_s
{
  /* Timer address */
  uintptr_t addr;
#ifdef CONFIG_DEVICE_IRQ
  /* Interrupt endpoints */
  struct dev_irq_src_s irq_eps[4];
  /* Request queue */
  dev_request_pqueue_root_t queue;
  uint32_t skew;
#endif
});

static uint64_t get_timer_value(struct bcm283x_systimer_private_s *pv)
{
  uint32_t lo, hi;

  hi = cpu_mem_read_32(pv->addr + BCM283X_SYSTIMER_CHIGH);
  lo = cpu_mem_read_32(pv->addr + BCM283X_SYSTIMER_CLOW);

  if (hi != cpu_mem_read_32(pv->addr + BCM283X_SYSTIMER_CHIGH))
    lo = cpu_mem_read_32(pv->addr + BCM283X_SYSTIMER_CLOW);

  return ((uint64_t)endian_le32(hi) << 32) | endian_le32(lo);
}

#ifdef CONFIG_DEVICE_IRQ

static void set_timer_compare(struct bcm283x_systimer_private_s *pv, dev_timer_value_t deadline)
{
  uint32_t skew = pv->skew;

  while (1)
    {
      uint64_t now = get_timer_value(pv);

      if (deadline > now)
        {
          cpu_mem_write_32(pv->addr + BCM283X_SYSTIMER_STATUS, endian_le32(1 << BCM283X_SYSTIMER_CMP_CHANNEL));
          cpu_mem_write_32(pv->addr + BCM283X_SYSTIMER_CMP(BCM283X_SYSTIMER_CMP_CHANNEL), endian_le32(deadline));

          if (deadline > get_timer_value(pv))
            break;
        }

      deadline = now + skew;
      pv->skew = skew;
      skew *= 2;
    }
}

static DEV_IRQ_SRC_PROCESS(bcm283x_systimer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct bcm283x_systimer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t status = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_SYSTIMER_STATUS))
        & (1 << BCM283X_SYSTIMER_CMP_CHANNEL);

      if (!status)
        break;

      cpu_mem_write_32(pv->addr + BCM283X_SYSTIMER_STATUS, endian_le32(status));
      struct dev_timer_rq_s *rq;

      while ((rq = dev_timer_rq_head(&pv->queue)))
        {
          if (rq->deadline > get_timer_value(pv))
            break;

          struct dev_timer_rq_s *next = dev_timer_rq_next(&pv->queue, rq);
          dev_timer_rq_remove(&pv->queue, rq);
          rq->base.drvdata = NULL;

          if (next != NULL)
            {
              rq = next;
              if (rq->deadline > get_timer_value(pv))
                cpu_mem_write_32(pv->addr + BCM283X_SYSTIMER_CMP(BCM283X_SYSTIMER_CMP_CHANNEL),
                                 endian_le32(rq->deadline));
            }

          lock_release(&dev->lock);
          dev_timer_rq_done(rq);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(bcm283x_systimer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct bcm283x_systimer_private_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == pv)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_timer_rq_prev(&pv->queue, rq) == NULL);

      if (first)
        rqnext = dev_timer_rq_next(&pv->queue, rq);

      dev_timer_rq_remove(&pv->queue, rq);
      rq->base.drvdata = NULL;

      if (rqnext)
        set_timer_compare(pv, rqnext->deadline);

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_TIMER_REQUEST(bcm283x_systimer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct bcm283x_systimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != 1)
    err = -EAGAIN;
  else
    {
      uint64_t value = get_timer_value(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          dev_timer_rq_insert(&pv->queue, rq);
          rq->base.drvdata = pv;

          if (dev_timer_rq_prev(&pv->queue, rq) == NULL)
            set_timer_compare(pv, rq->deadline);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_TIMER_GET_VALUE(bcm283x_systimer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct bcm283x_systimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rev && rev != 1)
    err = -EAGAIN;
  else
    *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CONFIG(bcm283x_systimer_config)
{
  error_t err = 0;

  if (cfg && device_get_res_freq(accessor->dev, &cfg->freq, 0))
    cfg->freq = DEV_FREQ_INVALID;

  if (res > 1)
    err = -ERANGE;

  if (cfg)
    {
      cfg->rev = 1;
      cfg->res = 1;
      cfg->cap = DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE;
      cfg->max = 0xffffffffffffffffULL;
#ifdef CONFIG_DEVICE_IRQ
      cfg->cap |= DEV_TIMER_CAP_REQUEST;
#endif
    }

  return err;
}

/************************************************************************/

#define bcm283x_systimer_use dev_use_generic

static DEV_INIT(bcm283x_systimer_init)
{
  struct bcm283x_systimer_private_s  *pv;


  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct bcm283x_systimer_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, pv->irq_eps, 4,
                         bcm283x_systimer_irq);

  if (device_irq_source_link(dev, pv->irq_eps, 4, 1 << BCM283X_SYSTIMER_CMP_CHANNEL))
    goto err_mem;

  dev_rq_pqueue_init(&pv->queue);
  pv->skew = 1;
#endif


  return 0;
#ifdef CONFIG_DEVICE_IRQ
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(bcm283x_systimer_cleanup)
{
  struct bcm283x_systimer_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  dev_rq_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, pv->irq_eps, 4);
#endif

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(bcm283x_systimer_drv, 0, "BCM283X system timer", bcm283x_systimer,
               DRIVER_TIMER_METHODS(bcm283x_systimer));

DRIVER_REGISTER(bcm283x_systimer_drv);

