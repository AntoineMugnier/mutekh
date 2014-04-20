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

    Copyright (c) 2013 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include "driver_m.h"

#include <mutek/kroutine.h>

/*
  The Arm-m profile may provide a cycle counter and a systick timer.
  This driver provides 3 device interfaces to access these timers:

  - Interface 0 exposes the value of the SysTick counter directly. The
    value is 24 bits wide if irq support is disabled and 64 bits if
    irq support is enabled. This interface is not able to handle
    requests.

  - Interface 1 use the SysTick decreasing counter as a prescaler to
    implement a 64 bits software timer. It is only available if irq
    support is enabled. This interface is able to handle requests.

  - Interface 2 exposes the value of the DWT_CYCCNT 32 bits cycle
    counter register. It is only available on some ARMv7m
    implementations. This interface is not able to handle requests.

  Timer start can not be performed on interface 0 and 1 at the same time.
*/

#if defined(CONFIG_CPU_ARM_TIMER_SYSTICK) && defined(CONFIG_DEVICE_IRQ)
void arm_timer_systick_irq(struct device_s *dev)
{
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if (!(cpu_mem_read_32(ARM_M_SYSTICK_CSR_ADDR) & ARM_M_SYSTICK_CSR_CNTFLAG))
    return;

  lock_spin(&dev->lock);
  pv->systick_value++;

  while (pv->systick_start & 1)
    {
      struct dev_timer_rq_s *rq = dev_timer_queue_head(&pv->systick_queue);

      if (rq == NULL)
        {
          /* stop timer */
          pv->systick_start &= ~1;
          if (pv->systick_start == 0)
            cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, ARM_M_SYSTICK_CSR_CLKSRC);
          break;
        }

      if (rq->deadline > pv->systick_value)
        break;

      rq->drvdata = NULL;
      dev_timer_queue_pop(&pv->systick_queue);

      lock_release(&dev->lock);
      kroutine_exec(&rq->kr, 0);
      lock_spin(&dev->lock);
    }

  lock_release(&dev->lock);
}
#endif

static DEVTIMER_REQUEST(arm_timer_request)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

  switch (tdev->number)
    {
#if defined(CONFIG_CPU_ARM_TIMER_SYSTICK) && defined(CONFIG_DEVICE_IRQ)
    case 1: {
      error_t err = 0;

      rq->tdev = tdev;
      LOCK_SPIN_IRQ(&dev->lock);

      if (pv->systick_start < 0)  /* hardware timer already used in mode 0 */
        err = -EBUSY;
      else
        {
          uint64_t val = pv->systick_value;

          if (rq->delay)
            rq->deadline = val + rq->delay;

          if (rq->deadline <= val)
            err = ETIMEDOUT;
          else
            {
              dev_timer_queue_insert(&pv->systick_queue, rq);
              rq->drvdata = pv;

              /* start timer if needed */
              if (pv->systick_start == 0)
                {
                  cpu_mem_write_32(ARM_M_SYSTICK_RVR_ADDR, pv->systick_period);
                  cpu_mem_write_32(ARM_M_SYSTICK_CVR_ADDR, 0);
                  cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, ARM_M_SYSTICK_CSR_CLKSRC |
                                   ARM_M_SYSTICK_CSR_ENABLE | ARM_M_SYSTICK_CSR_TICKINT);
                }
              pv->systick_start |= 1;
            }
        }

      LOCK_RELEASE_IRQ(&dev->lock);

      return err;
    }
#endif

    default:
      return -ENOTSUP;
    }
}

static DEVTIMER_CANCEL(arm_timer_cancel)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

  switch (tdev->number)
    {
#if defined(CONFIG_CPU_ARM_TIMER_SYSTICK) && defined(CONFIG_DEVICE_IRQ)
    case 1: {
      error_t err = 0;

      assert(rq->tdev->dev == dev && rq->tdev->number == 1);

      LOCK_SPIN_IRQ(&dev->lock);

      if (rq->drvdata == pv)
        {
          assert(pv->systick_start & 1);

          rq->drvdata = NULL;
          dev_timer_queue_remove(&pv->systick_queue, rq);

          /* stop timer if not in use */
          if (dev_timer_queue_isempty(&pv->systick_queue))
            {
              pv->systick_start &= ~1;
              if (pv->systick_start == 0)
                cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, ARM_M_SYSTICK_CSR_CLKSRC);
            }
        }
      else
        {
          err = -ETIMEDOUT;
        }

      LOCK_RELEASE_IRQ(&dev->lock);

      return err;
    }
#endif

    default:
      return -ENOTSUP;
    }
}

static DEVTIMER_START_STOP(arm_timer_start_stop)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (tdev->number)
    {
#ifdef CONFIG_CPU_ARM_TIMER_SYSTICK
    case 0:
    case 1: {
      uint_fast8_t mode = tdev->number;
      int_fast8_t st = mode ? 2 : -2;
      if (pv->systick_start && ((pv->systick_start > 0) ^ mode))
        {
          err = -EBUSY;
        }
      else if (start)
        {
          if (pv->systick_start == 0)
            {
              pv->systick_value = 0;
              uint32_t top = 0xffffff;
# ifdef CONFIG_DEVICE_IRQ
              if (mode)
                top = pv->systick_period;
# endif
              cpu_mem_write_32(ARM_M_SYSTICK_RVR_ADDR, top);
              cpu_mem_write_32(ARM_M_SYSTICK_CVR_ADDR, 0);

# ifdef CONFIG_DEVICE_IRQ
              cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, ARM_M_SYSTICK_CSR_CLKSRC |
                               ARM_M_SYSTICK_CSR_ENABLE | ARM_M_SYSTICK_CSR_TICKINT);
# else
              cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, ARM_M_SYSTICK_CSR_ENABLE | ARM_M_SYSTICK_CSR_CLKSRC);
# endif
            }
          pv->systick_start += st;
        }
      else
        {
          if ((pv->systick_start & ~1) == 0)
            err = -EINVAL;
          else
            {
              pv->systick_start -= st;
              if (pv->systick_start == 0)
                cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, ARM_M_SYSTICK_CSR_CLKSRC);
            }
        }
      break;
    }
#endif

#ifdef CONFIG_CPU_ARM_TIMER_DWTCYC
    case 2: {
      uint32_t ctrl = cpu_mem_read_32(ARM_M_DWT_CTRL_ADDR);
      if (ctrl & ARM_M_DWT_CTRL_NOCYCCNT)
        err = -ENOTSUP;
      else if (start)
        {
          if (pv->dwt_cycnt_start++ == 0)
            cpu_mem_write_32(ARM_M_DWT_CTRL_ADDR, ctrl | ARM_M_DWT_CTRL_CYCCNTENA);
        }
      else
        {
          if (--pv->dwt_cycnt_start == 0)
            cpu_mem_write_32(ARM_M_DWT_CTRL_ADDR, ctrl & ~ARM_M_DWT_CTRL_CYCCNTENA);
        }
      break;
    }
#endif

    default:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(arm_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (tdev->number)
    {
#ifdef CONFIG_CPU_ARM_TIMER_SYSTICK
    case 0:
      if (!pv->systick_start)
        err = -EBUSY;
      else
        {
          uint64_t v = cpu_mem_read_32(ARM_M_SYSTICK_CVR_ADDR) ^ 0xffffff;

# ifdef CONFIG_DEVICE_IRQ
          if (v < 0x800000)
            if (cpu_mem_read_32(ARM_M_SYSTICK_CSR_ADDR) & ARM_M_SYSTICK_CSR_CNTFLAG)
              pv->systick_value++;
          v += pv->systick_value << 24;
# endif
          *value = v;
        }
      break;

# ifdef CONFIG_DEVICE_IRQ
    case 1:
      if (!pv->systick_start)
        err = -EBUSY;
      else
        *value = pv->systick_value;
      break;
# endif
#endif

#ifdef CONFIG_CPU_ARM_TIMER_DWTCYC
    case 2: {          /* cycle counter */
      uint32_t ctrl = cpu_mem_read_32(ARM_M_DWT_CTRL_ADDR);
      if (ctrl & ARM_M_DWT_CTRL_NOCYCCNT)
        err = -ENOTSUP;
      else if (ctrl & ARM_M_DWT_CTRL_CYCCNTENA)
        *value = cpu_mem_read_32(ARM_M_DWT_CYCCNT_ADDR);
      else
        err = -EBUSY;
      break;
    }
#endif

    default:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_RESOLUTION(arm_timer_resolution)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (tdev->number)
    {
#ifdef CONFIG_CPU_ARM_TIMER_SYSTICK
    case 0:    /* systick as a free running counter */
      if (res)
        {
          if (*res != 0)
            err = -ENOTSUP;
          *res = 1;
        }
      if (max)
# ifdef CONFIG_DEVICE_IRQ
        *max = 0xffffffffffffffffULL;
# else
        *max = 0xffffff;
# endif
      break;

# ifdef CONFIG_DEVICE_IRQ
    case 1:     /* systick as a prescaler */
      if (res)
        {
          if (*res)
            {
              if (pv->systick_start)
                {
                  err = -EBUSY;
                }
              else if (*res < ARM_M_SYSTICK_MIN_PERIOD)
                {
                  pv->systick_period = ARM_M_SYSTICK_MIN_PERIOD;
                  err = -ERANGE;
                }
              else
                {
                  pv->systick_period = *res;
                }
            }
          *res = pv->systick_period;
        }

      if (max)
        *max = 0xffffffffffffffffULL;
      break;
# endif
#endif

#ifdef CONFIG_CPU_ARM_TIMER_DWTCYC
    case 2:    /* cycle counter */
      if (res)
        {
          if (*res != 0)
            err = -ENOTSUP;
          *res = 1;
        }

      if (max)
        *max = 0xffffffff;
      break;
#endif

    default:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

const struct driver_timer_s  arm_m_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_request       = arm_timer_request,
  .f_cancel        = arm_timer_cancel,
  .f_start_stop    = arm_timer_start_stop,
  .f_get_value     = arm_timer_get_value,
  .f_resolution    = arm_timer_resolution,
};

