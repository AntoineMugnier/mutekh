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
#include <device/clock.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <arch/pic32/timer.h>
#include <arch/pic32/oc.h>
#include <arch/pic32/freq.h>

#define PIC32_TIMER_HW_WIDTH 32
#define PIC32_TIMER_HW_MASK  0xFFFFFFFF
#define PIC32_TIMER_SW_MASK  0xffffffff00000000ULL
#define PIC32_TIMER_TOP      0xFFFFFFFF

 
DRIVER_PV(struct pic32_timer_private_s
{
  /* Last known msb of hw timer value */
  bool_t    t31; 
  /* Timer address */
  uintptr_t taddr;
  /* Compare output address */
  uintptr_t caddr[2];
  /* Start timer counter, bit 0 indicates if there are pending requests */
  /* Timer Software value */
  uint64_t swvalue;
  /* Interrupt endpoint */
  struct dev_irq_src_s irq_ep[3];
  /* Request queue */
  dev_request_pqueue_root_t queue;
  enum dev_timer_capabilities_e cap:8;
  dev_timer_cfgrev_t rev;
  /* Core frequency */
  struct dev_freq_s freq;
  /* cpu irq save mask */
  cpu_irq_state_t irq_save;
});

/* This function returns a concatenation of the software timer value and 
   of the hardware timer value. If a top value overflow interrupt is pending
   in the timer, the software timer value is incremented by one to get the 
   most recent timer value. */

static dev_timer_value_t get_timer_value(struct pic32_timer_private_s *pv)
{
  dev_timer_value_t v = endian_le32(cpu_mem_read_32(pv->taddr + PIC32_TIMER_TMR_ADDR));

  /* A counter wrap has just occured */
  bool_t d = pv->t31 && !((v >> 31) & 1); 

  v += (pv->swvalue << PIC32_TIMER_HW_WIDTH);

  if (d)
    return v + (1ULL << PIC32_TIMER_HW_WIDTH);

  return v;
}

/* This function starts the hardware timer counter. */
static inline void pic32_timer_start_counter(struct pic32_timer_private_s *pv)
{
  uint32_t x = endian_le32(PIC32_TIMER_CON_ON | PIC32_TIMER_CON_T32);
  cpu_mem_write_32(pv->taddr + PIC32_TIMER_CON_ADDR, x);
}

/* This function stops the hardware timer counter. */
static inline void pic32_timer_stop_counter(struct pic32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->taddr + PIC32_TIMER_CON_CLR_ADDR, PIC32_TIMER_CON_ON);
}

/* This function writes a value in the compare module associated to the 
   timer. When the timer counter value will be equal to this value a compare
   interrup will be raised. */

static inline void pic32_timer_enable_compare(struct pic32_timer_private_s *pv, dev_timer_value_t v)
{

  cpu_mem_write_32(pv->caddr[1] + PIC32_OUTPUT_COMPARE_CON_ADDR, 0);

  cpu_mem_write_32(pv->caddr[1] + PIC32_OUTPUT_COMPARE_OCR_ADDR, endian_le32(v));

  /* One shot compare */

  uint32_t x = PIC32_OUTPUT_COMPARE_CON_ON | PIC32_OUTPUT_COMPARE_CON_OC32 |
               PIC32_OUTPUT_COMPARE_CON_OCM(LEVEL_HIGH);

  cpu_mem_write_32(pv->caddr[1] + PIC32_OUTPUT_COMPARE_CON_ADDR, endian_le32(x));

}

/* This function disables the compare channel. */

static inline void pic32_timer_disable_compare(struct pic32_timer_private_s *pv)
{
  cpu_mem_write_32(pv->caddr[1] + PIC32_OUTPUT_COMPARE_CON_ADDR, 0);
}

static bool_t pic32_timer_request_start(struct pic32_timer_private_s *pv,
                                      struct dev_timer_rq_s *rq,
                                      dev_timer_value_t value)
{
  /* enable hw comparator if software part of the counter match */
  if (((rq->deadline ^ value) & PIC32_TIMER_SW_MASK))
    return 0;

  pic32_timer_enable_compare(pv, rq->deadline);

  /* hw compare for == only, check for race condition */
  if (rq->deadline <= get_timer_value(pv))
    return 1;

  return 0;
}

static void pic32_timer_test_queue(struct device_s *dev)
{
  struct pic32_timer_private_s *pv = dev->drv_pv;

  while (1)
    {
      struct dev_timer_rq_s *rq = dev_timer_rq_head(&pv->queue);

      if (rq == NULL)
        {
          dev->start_count &= ~1;
          if (dev->start_count == 0)
            pic32_timer_stop_counter(pv);
          break;
        }

      uint64_t value = get_timer_value(pv);

      /* setup compare for first request */
      if (rq->deadline > value)
        if (!pic32_timer_request_start(pv, rq, value))
          break;

      dev_timer_rq_remove(&pv->queue, rq);
      pic32_timer_disable_compare(pv);
      rq->base.drvdata = NULL;

      lock_release_irq2(&dev->lock, &pv->irq_save);
      dev_timer_rq_done(rq);
      lock_spin_irq2(&dev->lock, &pv->irq_save);
    }
}

static DEV_IRQ_SRC_PROCESS(pic32_timer_midle_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  pv->t31 = 1;

  lock_release_irq2(&dev->lock, &pv->irq_save);
}

static DEV_IRQ_SRC_PROCESS(pic32_timer_match_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  pic32_timer_disable_compare(pv);
  pic32_timer_test_queue(dev);

  lock_release_irq2(&dev->lock, &pv->irq_save);
}

static DEV_IRQ_SRC_PROCESS(pic32_timer_ovf_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  pv->t31 = 0;
  pv->swvalue++;

  pic32_timer_test_queue(dev);

  lock_release_irq2(&dev->lock, &pv->irq_save);
}

static DEV_TIMER_CANCEL(pic32_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  if (rq->base.drvdata == pv)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_timer_rq_prev(&pv->queue, rq) == NULL);

      if (first)
        rqnext = dev_timer_rq_next(&pv->queue, rq);

      dev_timer_rq_remove(&pv->queue, rq);
      rq->base.drvdata = NULL;

      /* Request was at top of queue */
      if (first)
        {
          pic32_timer_disable_compare(pv);

          if (rqnext != NULL)
            {
              /* start next request, raise irq on race condition */
              if (pic32_timer_request_start(pv, rqnext, get_timer_value(pv)))
                pic32_timer_test_queue(dev);
            }
          else
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                pic32_timer_stop_counter(pv);
            }
        }
      err = 0;
    }

  lock_release_irq2(&dev->lock, &pv->irq_save);

  return err;
}

static DEV_TIMER_REQUEST(pic32_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  if (rq->rev && rq->rev != pv->rev)
    err = -EAGAIN;
  else
    {
      /* Start timer if needed */
      if (dev->start_count == 0)
        pic32_timer_start_counter(pv);

      uint64_t value = get_timer_value(pv);

      if (rq->delay)
        rq->deadline = value + rq->delay;

      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          dev->start_count |= 1;
          dev_timer_rq_insert(&pv->queue, rq);
          rq->base.drvdata = pv;

          /* start request, raise irq on race condition */
          if (dev_timer_rq_prev(&pv->queue, rq) == NULL)
            if (pic32_timer_request_start(pv, rq, value))
              pic32_timer_test_queue(dev);
        }

      if (dev->start_count == 0)
        pic32_timer_stop_counter(pv);
    }

  lock_release_irq2(&dev->lock, &pv->irq_save);

  return err;
}

static DEV_USE(pic32_timer_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct pic32_timer_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        pic32_timer_start_counter(pv);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct pic32_timer_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        pic32_timer_stop_counter(pv);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_GET_VALUE(pic32_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  *value = get_timer_value(pv);

  lock_release_irq2(&dev->lock, &pv->irq_save);

  return 0;
}

static DEV_TIMER_CONFIG(pic32_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct pic32_timer_private_s *pv = dev->drv_pv;
  uint32_t ctrl, r, div;
 
  error_t err = 0;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  if (cfg)
    cfg->freq = pv->freq;

  if (res)
    {
      if (dev->start_count)
        {
          err = -EBUSY;
          r = res;
        }
      else
        {
          if (res >= 256)
            div = 7;
          else if (res >= 64)
            div = 6;
          else
            div = bit_msb_index(res);

          ctrl = endian_le32(cpu_mem_read_32(pv->taddr + PIC32_TIMER_CON_ADDR));
          PIC32_TIMER_CON_TCKPS_SETVAL(ctrl, div);
          cpu_mem_write_32(pv->taddr + PIC32_TIMER_CON_ADDR, endian_le32(ctrl));

          r = 1 << div;
          if (r != res)
            err = -ERANGE;

          pv->rev += 2;
        }
    }
  else
    {
      uint32_t ctrl = endian_le32(cpu_mem_read_32(pv->taddr + PIC32_TIMER_CON_ADDR));
      ctrl = PIC32_TIMER_CON_TCKPS_GET(ctrl);
      r = ctrl == 7 ? 256 : 1 << ctrl;
    }

  if (cfg)
    {
      cfg->rev = pv->rev;
      cfg->res = r;
      cfg->cap = pv->cap;
      cfg->max = 0xffffffffffffffffULL;
    }

  lock_release_irq2(&dev->lock, &pv->irq_save);

  return err;
}

/************************************************************************/


static DEV_INIT(pic32_timer_init)
{
  struct pic32_timer_private_s  *pv;


  pv = mem_alloc(sizeof(struct pic32_timer_private_s), (mem_scope_sys));

  memset(pv, 0, sizeof(*pv));

  if (!pv)
    return -ENOMEM;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->taddr, NULL))
    return -ENOENT;
  if (device_res_get_uint(dev, DEV_RES_MEM, 1, &pv->caddr[0], NULL))
    return -ENOENT;
  if (device_res_get_uint(dev, DEV_RES_MEM, 2, &pv->caddr[1], NULL))
    return -ENOENT;

  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

  pv->rev = 1;
  pv->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE;
  dev->drv_pv = pv;

  pv->cap |= DEV_TIMER_CAP_REQUEST;

  dev_rq_pqueue_init(&pv->queue);

  /* Stop timer  and  OC */
  cpu_mem_write_32(pv->taddr + PIC32_TIMER_CON_ADDR, 0);
  cpu_mem_write_32(pv->caddr[0] + PIC32_OUTPUT_COMPARE_CON_ADDR, 0);
  cpu_mem_write_32(pv->caddr[1] + PIC32_OUTPUT_COMPARE_CON_ADDR, 0);

  /* Configure output compare for midle counter */

  cpu_mem_write_32(pv->caddr[0] + PIC32_OUTPUT_COMPARE_OCR_ADDR, PIC32_TIMER_TOP/2);

  uint32_t x = PIC32_OUTPUT_COMPARE_CON_ON | PIC32_OUTPUT_COMPARE_CON_OC32 |
               PIC32_OUTPUT_COMPARE_CON_OCM(TOGGLE);

  cpu_mem_write_32(pv->caddr[0] + PIC32_OUTPUT_COMPARE_CON_ADDR, x);
  
  /* Configure Timer */

  cpu_mem_write_32(pv->taddr + PIC32_TIMER_PR_ADDR, PIC32_TIMER_TOP);

  /* init irq endpoints */
  device_irq_source_init(dev, &pv->irq_ep[0], 1, &pic32_timer_ovf_irq);
  device_irq_source_init(dev, &pv->irq_ep[1], 1, &pic32_timer_midle_irq);
  device_irq_source_init(dev, &pv->irq_ep[2], 1, &pic32_timer_match_irq);

  /* Enable only timer ovf irq */
  if (device_irq_source_link(dev, pv->irq_ep, 3, -1))
    goto err_mem;


  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pic32_timer_cleanup)
{
  struct pic32_timer_private_s *pv = dev->drv_pv;

  /* Stop timer */
  cpu_mem_write_32(pv->taddr + PIC32_TIMER_CON_ADDR, 0);
  cpu_mem_write_32(pv->caddr[0] + PIC32_OUTPUT_COMPARE_CON_ADDR, 0);
  cpu_mem_write_32(pv->caddr[1] + PIC32_OUTPUT_COMPARE_CON_ADDR, 0);

  dev_rq_pqueue_destroy(&pv->queue);
  device_irq_source_unlink(dev, pv->irq_ep, 3);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pic32_timer_drv, 0, "PIC32 Timer", pic32_timer,
               DRIVER_TIMER_METHODS(pic32_timer));

DRIVER_REGISTER(pic32_timer_drv);

