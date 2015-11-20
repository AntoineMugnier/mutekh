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

    Copyright (c) 2015 Vincent Defilippi <vincentdefilippi@gmail.com>

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

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/prcm.h>
#include <arch/cc26xx/gpt.h>

#define CC26XX_GPT_TIMER_A    0
#define CC26XX_GPT_TIMER_B    1

#define CC26XX_TIMER_HW_WIDTH 32
#define CC26XX_TIMER_HW_MASK  0xffffffff
#define CC26XX_TIMER_SW_MASK  0xffffffff00000000ULL
#define CC26XX_TIMER_TOP      0xffffffff

 
struct cc26xx_timer_private_s
{
  /* Timer address */
  uintptr_t addr;
#ifdef CONFIG_DEVICE_IRQ
  /* Timer Software value */
  uint64_t swvalue;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_eps;
  /* Request queue */
  dev_request_pqueue_root_t queue;
#endif

  struct dev_freq_s freq;
  struct dev_freq_accuracy_s acc;
  enum dev_timer_capabilities_e cap:8;
  dev_timer_cfgrev_t rev;
  /* Start timer counter, bit 0 indicates if there are pending requests */
  uint_fast8_t start_count;
};

/* This function starts the hardware timer counter. */
static inline void cc26xx_timer_start_counter(struct cc26xx_timer_private_s *pv)
{
  /* enable the timer */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_CTL_ADDR, endian_le32(
                    CC26XX_GPT_CTL_TAEN(EN) |
                    CC26XX_GPT_CTL_TASTALL(DIS)));
}

/* This function stops the hardware timer counter. */
static inline void cc26xx_timer_stop_counter(struct cc26xx_timer_private_s *pv)
{
  /* disable the timer */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_CTL_ADDR, endian_le32(
                    CC26XX_GPT_CTL_TAEN(DIS) |
                    CC26XX_GPT_CTL_TASTALL(DIS)));
}

/* This function returns a concatenation of the software timer value and 
   of the hardware timer value. If a top value overflow interrupt is pending
   in the timer, the software timer value is incremented by one to get the 
   most recent timer value. */
static uint64_t get_timer_value(struct cc26xx_timer_private_s *pv)
{
  uint64_t value = endian_le32(cpu_mem_read_32(pv->addr + CC26XX_GPT_TAV_ADDR));

#ifdef CONFIG_DEVICE_IRQ
  if (value < CC26XX_TIMER_HW_MASK / 2)      /* check if a wrap just occured */
    {
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + CC26XX_GPT_MIS_ADDR));
      if (x & CC26XX_GPT_MIS_TATOMIS)
        value += 1ULL << CC26XX_TIMER_HW_WIDTH;
    }
  return value + (pv->swvalue << CC26XX_TIMER_HW_WIDTH);
#else
  return value;
#endif
}

#ifdef CONFIG_DEVICE_IRQ

/* This function writes a value in the Comparator of the 
   timer. When the timer counter value will be greater than this value 
   a compare interrup will be raised. */
static inline void cc26xx_timer_enable_compare(struct cc26xx_timer_private_s *pv, dev_timer_value_t v)
{
  /* Write v in Comparator */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_TAMATCHR_ADDR, endian_le32(v));

  /* set interrupt mask */
  uint32_t imr = cpu_mem_read_32(pv->addr + CC26XX_GPT_IMR_ADDR);
  cpu_mem_write_32(pv->addr + CC26XX_GPT_IMR_ADDR, imr | CC26XX_GPT_IMR_TAMIM(EN));
}


/* This function disables the interrupt associated with the comparator */
static inline void cc26xx_timer_disable_compare(struct cc26xx_timer_private_s *pv)
{
  /* clear interrupt mask */
  uint32_t imr = cpu_mem_read_32(pv->addr + CC26XX_GPT_IMR_ADDR);
  cpu_mem_write_32(pv->addr + CC26XX_GPT_IMR_ADDR, imr & ~CC26XX_GPT_IMR_TAMIM(EN));
}

static bool_t cc26xx_timer_request_start(struct cc26xx_timer_private_s *pv,
                                      struct dev_timer_rq_s *rq,
                                      dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & CC26XX_TIMER_SW_MASK))
    return 0;

  cc26xx_timer_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_timer_value(pv))
    return 1;

  return 0;
}

static void cc26xx_timer_rq_handler(struct device_s *dev)
{
  struct cc26xx_timer_private_s *pv = dev->drv_pv;

  while (1)
    {
      struct dev_timer_rq_s *rq;
      rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));
      if (rq == NULL)
        {
          pv->start_count &= ~1;
          if (pv->start_count == 0)
            cc26xx_timer_stop_counter(pv);
          break;
        }

      uint64_t value = get_timer_value(pv);

      /* setup compare for first request */
      if (rq->deadline > value)
        if (!cc26xx_timer_request_start(pv, rq, value))
          break;

      dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
      cc26xx_timer_disable_compare(pv);
      rq->rq.drvdata = NULL;

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr);
      lock_spin(&dev->lock);
    }

}

static DEV_IRQ_SRC_PROCESS(cc26xx_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct cc26xx_timer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + CC26XX_GPT_MIS_ADDR))
        & (CC26XX_GPT_MIS_TATOMIS | CC26XX_GPT_MIS_TAMMIS);

      if (!irq)
        break;

      cpu_mem_write_32(pv->addr + CC26XX_GPT_ICLR_ADDR, irq);

      /* Compare interrupt */
      if (irq & CC26XX_GPT_MIS_TAMMIS)
        cc26xx_timer_disable_compare(pv);

      /* Update the software part of the counter */
      if (irq & CC26XX_GPT_MIS_TATOMIS)
        pv->swvalue++;

      cc26xx_timer_rq_handler(dev);
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_CANCEL(cc26xx_timer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct cc26xx_timer_private_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata == pv)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_request_pqueue_prev(&pv->queue, dev_timer_rq_s_base(rq)) == NULL);

      if (first)
        rqnext = dev_timer_rq_s_cast(dev_request_pqueue_next(&pv->queue, dev_timer_rq_s_base(rq)));

      dev_timer_pqueue_remove(&pv->queue, dev_timer_rq_s_base(rq));
      rq->rq.drvdata = NULL;

      if (first)
        {
          cc26xx_timer_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
              if (cc26xx_timer_request_start(pv, rqnext, get_timer_value(pv)))
                cc26xx_timer_rq_handler(dev);
            }
          else
            {
              pv->start_count &= ~1;
              if (pv->start_count == 0)
                cc26xx_timer_stop_counter(pv);
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

static DEV_TIMER_REQUEST(cc26xx_timer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct cc26xx_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      /* Start timer if needed */
      if (pv->start_count == 0)
        cc26xx_timer_start_counter(pv);

      uint64_t value = get_timer_value(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          pv->start_count |= 1;
          dev_timer_pqueue_insert(&pv->queue, dev_timer_rq_s_base(rq));
          rq->rq.drvdata = pv;

          /* start request, raise irq on race condition */
          if (dev_request_pqueue_prev(&pv->queue, dev_timer_rq_s_base(rq)) == NULL)
            if (cc26xx_timer_request_start(pv, rq, value))
              cc26xx_timer_rq_handler(dev);
        }

      if (pv->start_count == 0)
        cc26xx_timer_stop_counter(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_USE(cc26xx_timer_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      if (accessor->number)
        return -ENOTSUP;
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_START:
    case DEV_USE_STOP:
      break;
    default:
      return -ENOTSUP;
    }

  struct device_s *dev = accessor->dev;
  struct cc26xx_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (op == DEV_USE_START)
    {
      if (pv->start_count == 0)
        cc26xx_timer_start_counter(pv);
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
            cc26xx_timer_stop_counter(pv);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(cc26xx_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_timer_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = get_timer_value(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(cc26xx_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_timer_private_s *pv = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg)
    {
      cfg->rev = pv->rev;
      cfg->res = 1;
      cfg->cap = pv->cap;
      cfg->freq.num = pv->freq.num;
      cfg->freq.denom = pv->freq.denom;
      cfg->acc.e = 0;
#ifdef CONFIG_DEVICE_IRQ
      cfg->max = 0xffffffffffffffffULL;
#else
      cfg->max = 0xffffffff;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/

static DEV_INIT(cc26xx_timer_init);
static DEV_CLEANUP(cc26xx_timer_cleanup);

DRIVER_DECLARE(cc26xx_timer_drv, 0, "CC26XX TIMER", cc26xx_timer,
               DRIVER_TIMER_METHODS(cc26xx_timer));

DRIVER_REGISTER(cc26xx_timer_drv);

static void power_domain_on(void)
{
  uint32_t reg;

  //peripheral power domain on
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR);
  reg |= CC26XX_PRCM_PDCTL0PERIPH_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR, reg);

  //waiting for power
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDSTAT0_ADDR) &
    CC26XX_PRCM_PDSTAT0_SERIAL_ON));
}

static void clk_enable(void)
{
  uint32_t reg;

  //gpt clock enable
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_GPTCLKGR_ADDR);
  reg |= CC26XX_PRCM_GPTCLKGR_CLK_EN;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_GPTCLKGR_ADDR, reg);

  //loading clocks modif
  reg = CC26XX_PRCM_CLKLOADCTL_LOAD;
  cpu_mem_write_32(CC26XX_PRCM_NONBUF_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR, reg);

  //waiting for clocks
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR) &
    CC26XX_PRCM_CLKLOADCTL_LOAD_DONE));
}

static DEV_INIT(cc26xx_timer_init)
{
  struct cc26xx_timer_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  power_domain_on();
  clk_enable();

  pv = mem_alloc(sizeof(struct cc26xx_timer_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;
  pv->start_count = 0;
  pv->rev = 1;
  pv->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE;
  dev->drv_pv = pv;

  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

#ifdef CONFIG_DEVICE_IRQ
  pv->cap |= DEV_TIMER_CAP_REQUEST;

  device_irq_source_init(dev, &pv->irq_eps, 1, cc26xx_timer_irq);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_mem;

  dev_request_pqueue_init(&pv->queue);
#else
  pv->cap |= DEV_TIMER_CAP_TICKLESS;
#endif

  /* Stop timer */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_CTL_ADDR, 0);

#ifdef CONFIG_DEVICE_IRQ
  /* Clear interrupts */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_ICLR_ADDR, -1);

  /* Enable Overflow interrupts */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_IMR_ADDR, CC26XX_GPT_IMR_TATOIM(EN));

  pv->swvalue = 0;
#else
  cpu_mem_write_32(pv->addr + CC26XX_GPT_IMR_ADDR, 0);
#endif

  /* Ctrl register configuration */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_TNMR_ADDR(0),
                    CC26XX_GPT_TNMR_TNMR(PERIODIC) |
                    CC26XX_GPT_TNMR_TNCDIR(UP) |
                    CC26XX_GPT_TNMR_TNMIE(EN) |
                    CC26XX_GPT_TNMR_TNWOT(NOWAIT) |
                    CC26XX_GPT_TNMR_TNMRSU(CYCLEUPDATE) |
                    CC26XX_GPT_TNMR_TNCINTD(EN_TO_INTR));

  /* 32-bits timer */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_CFG_ADDR,
                    CC26XX_GPT_CFG_CFG(32BIT_TIMER));

  /* Set counter wrapping value to CC26XX_TIMER_TOP */
  cpu_mem_write_32(pv->addr + CC26XX_GPT_TAILR_ADDR, endian_le32(CC26XX_TIMER_TOP));

  dev->drv = &cc26xx_timer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(cc26xx_timer_cleanup)
{
  struct cc26xx_timer_private_s *pv = dev->drv_pv;

  if (pv->start_count & 1)
    return -EBUSY;

  /* Stop timer */
  cc26xx_timer_stop_counter(pv);

#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);
#endif

  mem_free(pv);

  return 0;
}

