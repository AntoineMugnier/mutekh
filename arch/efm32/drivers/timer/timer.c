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

    Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

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

#include <arch/efm32_timer.h>

#define EFM32_TIMER_HW_WIDTH 16
#define EFM32_TIMER_HW_MASK  0xffff
#define EFM32_TIMER_SW_MASK  0xffffffffffff0000ULL
#define EFM32_TIMER_TOP      0xffff
#define EFM32_TIMER_CHANNEL  0

 
struct efm32_timer_private_s
{
  /* Timer address */
  uintptr_t addr;
  /* Start timer counter, bit 0 indicates if there are pending requests */
  uint_fast8_t start_count;
#ifdef CONFIG_DEVICE_IRQ
  /* Timer Software value */
  uint64_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_ep_s irq_eps;
  /* Request queue */
  dev_timer_queue_root_t queue;
#endif
};

/* This function starts the hardware timer counter. */
static inline void efm32_timer_start_counter(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_START));
}

/* This function stops the hardware timer counter. */
static inline void efm32_timer_stop_counter(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
}

/* This function returns a concatenation of the software timer value and 
   of the hardware timer value. If a top value overflow interrupt is pending
   in the timer, the software timer value is incremented by one to get the 
   most recent timer value. */
static uint64_t get_timer_value(struct efm32_timer_private_s *pv)
{
  uint64_t value = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_CNT_ADDR));

#ifdef CONFIG_DEVICE_IRQ
  if (value < EFM32_TIMER_HW_MASK / 2)      /* check if a wrap just occured */
    {
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR));
      if (x & EFM32_TIMER_IF_OF)
        value += 1ULL << EFM32_TIMER_HW_WIDTH;
    }

  return value + (pv->swvalue << EFM32_TIMER_HW_WIDTH);
#else
  return value;
#endif
}

#ifdef CONFIG_DEVICE_IRQ

/* This function writes a value in the Compare/Capture channel 0 of the 
   timer. When the timer counter value will be greater than this value 
   a compare interrup will be raised. */
static inline void efm32_timer_enable_compare(struct efm32_timer_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Compare 0 channel */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CCV_ADDR(EFM32_TIMER_CHANNEL),
                   endian_le32(v & EFM32_TIMER_CC_CCV_MASK));

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(EFM32_TIMER_CHANNEL),
                   endian_le32(EFM32_TIMER_CC_CTRL_MODE(OUTPUTCOMPARE)));
}

/* This function disables the interrupt associated to compare/capture
   channel 0. */
static inline void efm32_timer_disable_compare(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(EFM32_TIMER_CHANNEL), 0);
}

static bool_t efm32_timer_request_start(struct efm32_timer_private_s *pv,
                                      struct dev_timer_rq_s *rq,
                                      dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & EFM32_TIMER_SW_MASK))
    return 0;

  efm32_timer_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_timer_value(pv))
    return 1;

  return 0;
}

static inline void efm32_timer_raise_irq(struct efm32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IFS_ADDR,
                   endian_le32(EFM32_TIMER_IF_CC(EFM32_TIMER_CHANNEL)));
}

static DEV_IRQ_EP_PROCESS(efm32_timer_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR))
        & (EFM32_TIMER_IEN_OF | EFM32_TIMER_IEN_CC(EFM32_TIMER_CHANNEL));

      if (!irq)
        break;

      cpu_mem_write_32(pv->addr + EFM32_TIMER_IFC_ADDR, endian_le32(irq));

      /* Compare channel interrupt */ 
      if (irq & EFM32_TIMER_IF_CC(EFM32_TIMER_CHANNEL))
        efm32_timer_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & EFM32_TIMER_IF_OF)
        pv->swvalue++;

      struct dev_timer_rq_s *rq = dev_timer_queue_head(&pv->queue);

      while (rq != NULL)
        {
          uint64_t value = get_timer_value(pv);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!efm32_timer_request_start(pv, rq, value))
              break;

          dev_timer_queue_pop(&pv->queue);
          efm32_timer_disable_compare(pv);
          rq->drvdata = 0;

          lock_release(&dev->lock);
          kroutine_exec(&rq->kr, 0);
          lock_spin(&dev->lock);

          rq = dev_timer_queue_head(&pv->queue);
          if (rq == NULL)
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                efm32_timer_stop_counter(pv);
              break;
            }
        }

    }

  lock_release(&dev->lock);
}
#endif

static DEVTIMER_CANCEL(efm32_timer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
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
          efm32_timer_disable_compare(pv);
          rq0 = dev_timer_queue_head(&pv->queue);

          if (rq0 != NULL)
            {
              /* start next request, raise irq on race condition */
              if (efm32_timer_request_start(pv, rq0, get_timer_value(pv)))
                efm32_timer_raise_irq(pv);
            }
          else
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                efm32_timer_stop_counter(pv);
            }
        }

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEVTIMER_REQUEST(efm32_timer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  rq->tdev = tdev;

  LOCK_SPIN_IRQ(&dev->lock);

  /* Start timer if needed */
  if (pv->start_count == 0)
    efm32_timer_start_counter(pv);

  uint64_t value = get_timer_value(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value)
    err = ETIMEDOUT;
  else
    {
      pv->start_count |= 1;
      dev_timer_queue_insert_ascend(&pv->queue, rq);
      rq->drvdata = pv;

      /* start request, raise irq on race condition */
      if (dev_timer_queue_head(&pv->queue) == rq)
        if (efm32_timer_request_start(pv, rq, value))
          efm32_timer_raise_irq(pv);
    }

  if (pv->start_count == 0)
    efm32_timer_stop_counter(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEVTIMER_START_STOP(efm32_timer_state_start_stop)
{
  struct device_s *dev = tdev->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (start)
    {
      if (pv->start_count == 0)
        efm32_timer_start_counter(pv);
      pv->start_count += 2;
    }
  else
    {
      if (pv->start_count < 2)
        err = -EINVAL;
      else
        {
          pv->start_count -= 2;
          if (pv->start_count == 0)
            efm32_timer_stop_counter(pv);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(efm32_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVTIMER_RESOLUTION(efm32_timer_resolution)
{
  struct device_s *dev = tdev->dev;
  struct efm32_timer_private_s *pv = dev->drv_pv;
  uint32_t ctrl, r, div;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (*res != 0)
        {
          if (pv->start_count)
            {
              err = -EBUSY;
            }
          else
            {
              /* div is either set to maximum value 10 or rounded down to the nearest power of 2 */
	      div = *res > 1024 ? 10 : sizeof(__compiler_sint_t) * 8 - __builtin_clz(*res) - 1;            

              ctrl = endian_le32(cpu_mem_read_32(EFM32_TIMER_CTRL_ADDR));
              EFM32_TIMER_CTRL_PRESC_SETVAL(ctrl, div);
              cpu_mem_write_32(EFM32_TIMER_CTRL_ADDR, endian_le32(ctrl));

	      r = 1 << div;

              if (r != *res)
	        err = -ERANGE;
              *res = r;
            }
        }
      else
        {
          uint32_t ctrl = endian_le32(cpu_mem_read_32(pv->addr + EFM32_TIMER_CTRL_ADDR));
          *res = 1 << EFM32_TIMER_CTRL_PRESC_GET(ctrl);
        }
    }

  if (max)
#ifdef CONFIG_DEVICE_IRQ
    *max = 0xffffffffffffffffULL;
#else
    *max = 0xffff;
#endif

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

const struct driver_timer_s  efm32_timer_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = efm32_timer_request,
  .f_cancel       = efm32_timer_cancel,
  .f_start_stop   = efm32_timer_state_start_stop,
  .f_get_value    = efm32_timer_get_value,
  .f_get_freq     = dev_timer_drv_get_freq,
  .f_resolution   = efm32_timer_resolution,
};

/************************************************************************/

static DEV_INIT(efm32_timer_init);
static DEV_CLEANUP(efm32_timer_cleanup);

const struct driver_s  efm32_timer_drv =
{
  .desc           = "EFM32 Timer",
  .f_init         = efm32_timer_init,
  .f_cleanup      = efm32_timer_cleanup,

  .classes        = {
    &efm32_timer_timer_drv,
    0
  }
};

static DEV_INIT(efm32_timer_init)
{
  struct efm32_timer_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct efm32_timer_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->start_count = 0;
  dev->drv_pv = pv;
  
#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_eps, 1,
                         efm32_timer_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_mem;

  dev_timer_queue_init(&pv->queue);
#endif

  /* Stop timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

#ifdef CONFIG_DEVICE_IRQ
  /* Clear interrupts */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IFC_ADDR, endian_le32(EFM32_TIMER_IFC_MASK));
  
  /* Enable Overflow interrupts */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, endian_le32(EFM32_TIMER_IEN_OF |
                                               EFM32_TIMER_IEN_CC(EFM32_TIMER_CHANNEL)));

  /* Set Compare/Capture channel 0 to Output Compare */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(EFM32_TIMER_CHANNEL), 0);

  pv->swvalue = 0;
#else
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, 0);
#endif

  /* Ctrl register configuration */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR,
                   endian_le32(EFM32_TIMER_CTRL_MODE(UP) |
                               EFM32_TIMER_CTRL_CLKSEL(PRESCHFPERCLK) |
                               EFM32_TIMER_CTRL_SYNC(NONE) |
                               EFM32_TIMER_CTRL_RISEA(NONE) |
                               EFM32_TIMER_CTRL_FALLA(NONE) |
                               EFM32_TIMER_CTRL_PRESC(DIV1024)));

  /* Set counter wrapping value to EFM32_TIMER_TOP */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_TOP_ADDR, endian_le32(EFM32_TIMER_TOP));

  dev->drv = &efm32_timer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_DEVICE_IRQ
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(efm32_timer_cleanup)
{
  struct efm32_timer_private_s *pv = dev->drv_pv;

  /* Stop timer */
  efm32_timer_stop_counter(pv);

#ifdef CONFIG_DEVICE_IRQ
  dev_timer_queue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(efm32_timer_drv);

