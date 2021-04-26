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

#include "driver.h"

#include <cpu/arm32m/v7m.h>

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

#if defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK) && defined(CONFIG_DEVICE_IRQ)
void arm_timer_systick_irq(struct device_s *dev)
{
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if (!(cpu_mem_read_32(ARMV7M_SYST_CSR_ADDR) &
      ARMV7M_SYST_CSR_COUNTFLAG))
    return;

  lock_spin(&dev->lock);
  pv->systick_value++;

  while (pv->systick_start & 1)
    {
      struct dev_timer_rq_s *rq = dev_timer_rq_head(&pv->systick_queue);

      if (rq == NULL)
        {
          /* stop timer */
          pv->systick_start &= ~1;
          if (pv->systick_start == 0)
            cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR,
              ARMV7M_SYST_CSR_CLKSOURCE(CPU));
          break;
        }

      if (rq->deadline > pv->systick_value)
        break;

      rq->base.drvdata = NULL;
      dev_timer_rq_remove(&pv->systick_queue, rq);

      lock_release(&dev->lock);
      dev_timer_rq_done(rq);
      lock_spin(&dev->lock);
    }

  lock_release(&dev->lock);
}
#endif

DEV_TIMER_REQUEST(arm_timer_request)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (accessor->number)
    {
#if defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK) && defined(CONFIG_DEVICE_IRQ)
    case 1: {

      if (pv->systick_start < 0)  /* hardware timer already used in mode 0 */
        {
          err = -EBUSY;
          break;
        }

      if (rq->rev && rq->rev != pv->systick_rev)
        {
          err = -EAGAIN;
          break;
        }

      uint64_t val = pv->systick_value;

      if (rq->delay)
        rq->deadline = val + rq->delay;

      if (rq->deadline <= val)
        {
          err = -ETIMEDOUT;
          break;
        }

      dev_timer_rq_insert(&pv->systick_queue, rq);
      rq->base.drvdata = pv;

      /* start timer if needed */
      if (pv->systick_start == 0)
        {
          cpu_mem_write_32(ARMV7M_SYST_RVR_ADDR, pv->systick_period);
          cpu_mem_write_32(ARMV7M_SYST_CVR_ADDR, 0);
          cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR,
                           ARMV7M_SYST_CSR_CLKSOURCE(CPU) |
                           ARMV7M_SYST_CSR_ENABLE | ARMV7M_SYST_CSR_TICKINT);
        }
      pv->systick_start |= 1;
      break;
    }
#endif

    default:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

DEV_TIMER_CANCEL(arm_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

  switch (accessor->number)
    {
#if defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK) && defined(CONFIG_DEVICE_IRQ)
    case 1: {
      error_t err = 0;

      LOCK_SPIN_IRQ(&dev->lock);

      if (rq->base.drvdata == pv)
        {
          assert(pv->systick_start & 1);

          rq->base.drvdata = NULL;
          dev_timer_rq_remove(&pv->systick_queue, rq);

          /* stop timer if not in use */
          if (dev_rq_pqueue_isempty(&pv->systick_queue))
            {
              pv->systick_start &= ~1;
              if (pv->systick_start == 0)
                cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR,
                  ARMV7M_SYST_CSR_CLKSOURCE(CPU));
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

error_t arm_timer_systick_use(const struct device_accessor_s *accessor,
                              enum dev_use_op_e op)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  switch (accessor->number)
    {
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
    case 0:
    case 1: {
      uint_fast8_t mode = accessor->number;
      int_fast8_t st = mode ? 2 : -2;
      if (pv->systick_start && ((pv->systick_start > 0) ^ mode))
        {
          err = -EBUSY;
        }
      else if (op == DEV_USE_START)
        {
          if (pv->systick_start == 0)
            {
              pv->systick_value = 0;
              uint32_t top = 0xffffff;
# ifdef CONFIG_DEVICE_IRQ
              if (mode)
                top = pv->systick_period;
# endif
              cpu_mem_write_32(ARMV7M_SYST_RVR_ADDR, top);
              cpu_mem_write_32(ARMV7M_SYST_CVR_ADDR, 0);

# ifdef CONFIG_DEVICE_IRQ
              cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR,
                  ARMV7M_SYST_CSR_CLKSOURCE(CPU) |
                  ARMV7M_SYST_CSR_ENABLE | ARMV7M_SYST_CSR_TICKINT);
# else
              cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR,
                  ARMV7M_SYST_CSR_CLKSOURCE(CPU) |
                  ARMV7M_SYST_CSR_ENABLE);
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
                cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR,
                  ARMV7M_SYST_CSR_CLKSOURCE(CPU));
            }
        }
      break;
    }
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
    case 2: {
      uint32_t ctrl = cpu_mem_read_32(ARM_M_DWT_CTRL_ADDR);
      if (ctrl & ARM_M_DWT_CTRL_NOCYCCNT)
        err = -ENOTSUP;
      else if (op == DEV_USE_START)
        {
# if defined(CONFIG_CPU_ARM32M_DWT)
          pv->dwt_cycnt_start++;
# else
          if (pv->dwt_cycnt_start++ == 0)
            cpu_mem_write_32(ARM_M_DWT_CTRL_ADDR,
              ctrl | ARM_M_DWT_CTRL_CYCCNTENA);
# endif
        }
      else
        {
# if defined(CONFIG_CPU_ARM32M_DWT)
          pv->dwt_cycnt_start--;
# else
          if (--pv->dwt_cycnt_start == 0)
            cpu_mem_write_32(ARM_M_DWT_CTRL_ADDR,
              ctrl & ~ARM_M_DWT_CTRL_CYCCNTENA);
# endif
        }
      break;
    }
#endif

    default:
      err = -ENOTSUP;
    }

  return err;
}

DEV_TIMER_GET_VALUE(arm_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (accessor->number)
    {
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
    case 0:
      if (pv->systick_start >= 0)
        err = -EBUSY;
      else if (rev && rev != 1)
        err = -EAGAIN;
      else
        {
          uint64_t v = cpu_mem_read_32(ARMV7M_SYST_CVR_ADDR) ^ 0xffffff;

# ifdef CONFIG_DEVICE_IRQ
          if (v < 0x800000)
            if (cpu_mem_read_32(ARMV7M_SYST_CSR_ADDR) &
              ARMV7M_SYST_CSR_COUNTFLAG)
              pv->systick_value++;
          v += pv->systick_value << 24;
# endif
          *value = v;

        }
      break;

# ifdef CONFIG_DEVICE_IRQ
    case 1:
      if (pv->systick_start <= 0)
        err = -EBUSY;
      else if (rev && rev != pv->systick_rev)
        err = -EAGAIN;
      else
        *value = pv->systick_value;
      break;
# endif
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
    case 2: {          /* cycle counter */
      uint32_t ctrl = cpu_mem_read_32(ARM_M_DWT_CTRL_ADDR);
      if (ctrl & ARM_M_DWT_CTRL_NOCYCCNT)
        err = -ENOTSUP;
      else if (rev && rev != 1)
        err = -EAGAIN;
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

DEV_TIMER_CONFIG(arm_timer_config)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg)
    cfg->freq = pv->freq;

  switch (accessor->number)
    {
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
    case 0:    /* systick as a free running counter */
      if (res > 1)
        err = -ERANGE;
      if (cfg)
        {
          cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES;
# if defined(CONFIG_CPU_ARM32M_CLOCK) && defined(CONFIG_DEVICE_CLOCK_VARFREQ)
          if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
            cfg->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;
# endif
# ifdef CONFIG_DEVICE_IRQ
          cfg->max = 0xffffffffffffffffULL;
# else
          cfg->cap |= DEV_TIMER_CAP_TICKLESS;
          cfg->max = 0xffffff;
# endif
          cfg->rev = 1;
          cfg->res = 1;
        }
      break;

# ifdef CONFIG_DEVICE_IRQ
    case 1:     /* systick as a prescaler */
      if (res)
        {
          if (pv->systick_start)
            {
              err = -EBUSY;
            }
          else if (res < ARM_M_SYSTICK_MIN_PERIOD)
            {
              res = ARM_M_SYSTICK_MIN_PERIOD;
              err = -ERANGE;
            }
          else if (res > 0xffffff)
            {
              res = 0xffffff;
              err = -ERANGE;
            }
          pv->systick_period = res;
          pv->systick_rev += 2;
        }
      if (cfg)
        {
          cfg->max = 0xffffffffffffffffULL;
          cfg->rev = pv->systick_rev;
          cfg->res = pv->systick_period;
          cfg->cap = DEV_TIMER_CAP_REQUEST | DEV_TIMER_CAP_STOPPABLE;
# if defined(CONFIG_CPU_ARM32M_CLOCK) && defined(CONFIG_DEVICE_CLOCK_VARFREQ)
          if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
            cfg->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;
#endif
        }
      break;
# endif
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
    case 2:    /* cycle counter */
      if (res > 1)
        err = -ENOTSUP;
      if (cfg)
        {
          cfg->max = 0xffffffff;
          cfg->rev = 1;
          cfg->res = 1;
          cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES
            | DEV_TIMER_CAP_TICKLESS;
# if defined(CONFIG_CPU_ARM32M_CLOCK) && defined(CONFIG_DEVICE_CLOCK_VARFREQ)
          if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
            cfg->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;
#endif
        }
      break;
#endif

    default:
      err = -ENOTSUP;
    }

 err:
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

