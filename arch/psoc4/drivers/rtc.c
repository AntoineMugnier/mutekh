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
  License along with this program.  If not, see
  <http://www.gnu.org/licenses/>.

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/timer.h>
#include <device/class/cmu.h>

#include <arch/psoc4/srss.h>
#include <arch/psoc4/variant.h>

#define SRSS PSOC4_SRSS_ADDR

#define dprintk(...) do {} while(0)

DRIVER_PV(struct psoc4_rtc_context_s
{
  struct dev_irq_src_s irq_ep;
  dev_request_pqueue_root_t queue;
  struct dev_freq_s freq;

  dev_timer_value_t base;
  uint32_t last_set_ts;

  struct dev_clock_sink_ep_s clock_sink;
});

/*
  There is no LFCLK-based timer.  How are we supposed to track time in
  deep sleep then ?

  Cypress suggests to use Watchdog as a RTC in various forum posts.
  This is not a serious fit.  It is crap, actually.  Now, try to make
  it tickless, and that's a big ugly kludge.

  So here we are.  WDT has three counters, but chaining cannot be used
  in a proper way with working comparators.  WDT has 3 LFCLK cycles
  latency to set comparators.

  The only workable solution is to use CTR0 as a comparator for lower
  16 bits, and use CTR2 as an accumulator for bits 16-31.  Upper bits
  are tracked in software.  Bit 31 of CTR2 is reflected in pv->base,
  if there is a discrepancy between them, this is because CTR2 had a
  bit flip on bit 31, this allows to track carry.

  When setting a deadline:

  - If bits 16+ change between now and deadline, use CTR2 flip bits to
    wait for them to match;

  - If bits 16+ match, use CTR0 to compare;

  - Unless we are in the racy case where (deadline - now) < 4, where
    we'll use CTR2 bit 0 to tick and get more interrupts not to miss
    the deadline.


  For enable/disable, we have to wait for ENABLED(x) to equal
  ENABLE(x) before changing them again. Let's busy wait for now.
*/

static bool_t psoc4_rtc_enable_is_acked(void)
{
  uint32_t control = cpu_mem_read_32(SRSS + SRSS_WDT_CONTROL_ADDR);

  return (!!(control & SRSS_WDT_CONTROL_ENABLED(0)) == !!(control & SRSS_WDT_CONTROL_ENABLE(0)))
      && (!!(control & SRSS_WDT_CONTROL_ENABLED(2)) == !!(control & SRSS_WDT_CONTROL_ENABLE(2)));
}

static void psoc4_rtc_start(struct psoc4_rtc_context_s *pv)
{
  uint32_t config;
  uint32_t control;

  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_CLOCK);

  while (!psoc4_rtc_enable_is_acked())
    ;

  config = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);

  SRSS_WDT_CONFIG_BITS2_SET(config, 31);
  control = 0
    | SRSS_WDT_CONTROL_INT(0)
    | SRSS_WDT_CONTROL_INT(1)
    | SRSS_WDT_CONTROL_INT(2)
    | SRSS_WDT_CONTROL_ENABLE(0)
    | SRSS_WDT_CONTROL_ENABLE(2)
    ;

  cpu_mem_write_32(SRSS + SRSS_WDT_CONTROL_ADDR, control);
  cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, config);

  dprintk("%s control %08x config %08x\n",
         __FUNCTION__, control, config);
}

static void psoc4_rtc_stop(struct psoc4_rtc_context_s *pv)
{
  while (!psoc4_rtc_enable_is_acked());

  cpu_mem_write_32(SRSS + SRSS_WDT_CONTROL_ADDR, 0)
    ;

  dev_clock_sink_gate(&pv->clock_sink, DEV_CLOCK_EP_NONE);

  dprintk("%s\n", __FUNCTION__);
}

static dev_timer_value_t psoc4_rtc_value_get(struct psoc4_rtc_context_s *pv)
{
  uint32_t high = cpu_mem_read_32(SRSS + SRSS_WDT_CTRHIGH_ADDR);

  if (~high & (uint32_t)pv->base & bit(31))
    return pv->base + bit(31) + high;

  return pv->base | high;
}

static void psoc4_rtc_deadline_set(struct psoc4_rtc_context_s *pv,
                               dev_timer_value_t next_deadline)
{
  dev_timer_value_t value;
  uint32_t changing_bits;
  uint32_t rising_bits;
  uint32_t config;
  uint32_t control;
  uint32_t bit;

  do {
    config = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);

    // Work around the fact we cannot write twice in MATCH register
    // during the same LFCLK cycle
    if (pv->last_set_ts == cpu_mem_read_32(SRSS + SRSS_WDT_CTRHIGH_ADDR)) {
      SRSS_WDT_CONFIG_BITS2_SET(config, 0);
      SRSS_WDT_CONFIG_MODE0_SET(config, NOTHING);
      cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, config);
      return;
    }

    value = psoc4_rtc_value_get(pv);
    rising_bits = next_deadline & ~value;
    changing_bits = next_deadline ^ value;

    control = cpu_mem_read_32(SRSS + SRSS_WDT_CONTROL_ADDR);
    control &= ~SRSS_WDT_CONTROL_INT(0);
    control &= ~SRSS_WDT_CONTROL_INT(2);

    bit = bit_msb_index(rising_bits);

    if (next_deadline <= value + 3) {
      SRSS_WDT_CONFIG_BITS2_SET(config, 0);
      SRSS_WDT_CONFIG_MODE0_SET(config, NOTHING);
    } else if ((next_deadline - value) >> 32) {
      SRSS_WDT_CONFIG_BITS2_SET(config, 31);
      SRSS_WDT_CONFIG_MODE0_SET(config, NOTHING);
    } else if ((changing_bits & 0xffff0000) == 0) {
      SRSS_WDT_CONFIG_BITS2_SET(config, 31);
      SRSS_WDT_CONFIG_MODE0_SET(config, INT);
    } else {
      SRSS_WDT_CONFIG_BITS2_SET(config, bit);
      SRSS_WDT_CONFIG_MODE0_SET(config, NOTHING);
    }

    cpu_mem_write_32(SRSS + SRSS_WDT_MATCH_ADDR,
                     SRSS_WDT_MATCH_COUNTER0((uint16_t)next_deadline));
    cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, config);
    cpu_mem_write_32(SRSS + SRSS_WDT_CONTROL_ADDR, control);

    pv->last_set_ts = cpu_mem_read_32(SRSS + SRSS_WDT_CTRHIGH_ADDR);
  } while (value != psoc4_rtc_value_get(pv));

  dprintk("%s now %llx dl %llx up %08x:%d chg %08x\n",
         __FUNCTION__, value, next_deadline, rising_bits, bit, changing_bits);
  dprintk("%s ctrl %08x conf %08x m %08x\n",
         __FUNCTION__, control, config, cpu_mem_read_32(SRSS + SRSS_WDT_MATCH_ADDR));
}

static void psoc4_rtc_deadline_disable(struct psoc4_rtc_context_s *pv)
{
  uint32_t config;
  uint32_t control;

  control = cpu_mem_read_32(SRSS + SRSS_WDT_CONTROL_ADDR);
  config = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);

  SRSS_WDT_CONFIG_BITS2_SET(config, 31);
  control &= ~SRSS_WDT_CONTROL_INT(0);
  control &= ~SRSS_WDT_CONTROL_INT(2);
  SRSS_WDT_CONFIG_MODE0_SET(config, NOTHING);

  cpu_mem_write_32(SRSS + SRSS_WDT_CONTROL_ADDR, control);
  cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, config);

  dprintk("%s\n", __FUNCTION__);
}

static DEV_TIMER_REQUEST(psoc4_rtc_request)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_rtc_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;
  error_t err = 0;

  //  assert(rq->rq.drvdata == NULL);

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev_request_pqueue_isempty(&pv->queue)) {
    if (!dev->start_count)
      psoc4_rtc_start(pv);

    dev->start_count |= 1;
  }

  value = psoc4_rtc_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value) {
    err = -ETIMEDOUT;

    if (dev_request_pqueue_isempty(&pv->queue)) {
      dev->start_count &= ~1;

      if (!dev->start_count)
        psoc4_rtc_stop(pv);
    }
  } else {
    dev_timer_pqueue_insert(&pv->queue, &rq->rq);
    rq->rq.drvdata = pv;

    if (dev_request_pqueue_head(&pv->queue) == &rq->rq)
      psoc4_rtc_deadline_set(pv, rq->deadline);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CANCEL(psoc4_rtc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_rtc_context_s *pv = dev->drv_pv;
  error_t err = -ENOENT;
  struct dev_timer_rq_s *head;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata != pv)
    goto out;

  head = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));

  dev_timer_pqueue_remove(&pv->queue, &rq->rq);
  rq->rq.drvdata = NULL;

  if (rq == head) {
    head = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));

    if (head)
      psoc4_rtc_deadline_set(pv, head->deadline);
    else
      psoc4_rtc_deadline_disable(pv);
  }

  err = 0;

  if (dev_request_pqueue_isempty(&pv->queue)) {
    dev->start_count &= ~1;

    if (!dev->start_count)
      psoc4_rtc_stop(pv);
  }

 out:

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(psoc4_rtc_use)
{
  struct device_accessor_s *accessor = param;
  struct device_s *dev = accessor->dev;
  struct psoc4_rtc_context_s *pv = dev->drv_pv;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    if (!dev->start_count)
      psoc4_rtc_start(pv);
    break;

  case DEV_USE_STOP:
    if (!dev->start_count)
      psoc4_rtc_stop(pv);
    break;
  }

  return 0;
}

static DEV_TIMER_GET_VALUE(psoc4_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_rtc_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = psoc4_rtc_value_get(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_IRQ_SRC_PROCESS(psoc4_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct psoc4_rtc_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *rq;
  uint32_t high, control;

  lock_spin(&dev->lock);

  dprintk("%s %x %x\n", __FUNCTION__, cpu_mem_read_32(SRSS + SRSS_WDT_CTRLOW_ADDR),
         cpu_mem_read_32(SRSS + SRSS_WDT_CTRHIGH_ADDR));

  control = cpu_mem_read_32(SRSS + SRSS_WDT_CONTROL_ADDR);
  control |= SRSS_WDT_CONTROL_INT(0) | SRSS_WDT_CONTROL_INT(1) | SRSS_WDT_CONTROL_INT(2);
  cpu_mem_write_32(SRSS + SRSS_WDT_CONTROL_ADDR, control);

  high = cpu_mem_read_32(SRSS + SRSS_WDT_CTRHIGH_ADDR);
  if (~high & (uint32_t)pv->base & bit(31))
    pv->base += bit(31);

  while ((rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue)))) {
    dev_timer_value_t value = psoc4_rtc_value_get(pv);

    dprintk("%s now %llx\n", __FUNCTION__, value);

    if (value < rq->deadline)
      break;

    dev_request_pqueue_pop(&pv->queue);
    rq->rq.drvdata = 0;

    lock_release(&dev->lock);
    kroutine_exec(&rq->rq.kr);
    lock_spin(&dev->lock);
  }

  if (rq) {
    psoc4_rtc_deadline_set(pv, rq->deadline);
  } else {
    dev->start_count &= ~1;
    psoc4_rtc_deadline_disable(pv);
    if (!dev->start_count)
      psoc4_rtc_stop(pv);
  }

  lock_release(&dev->lock);
}

static DEV_TIMER_CONFIG(psoc4_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_rtc_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq = pv->freq;
    cfg->rev = 0;
    cfg->res = 1;
    cfg->cap = DEV_TIMER_CAP_STOPPABLE
      | DEV_TIMER_CAP_HIGHRES
      | DEV_TIMER_CAP_TICKLESS
      | DEV_TIMER_CAP_KEEPVALUE
      | DEV_TIMER_CAP_REQUEST;
    cfg->max = -1;
  }

  if (res > 1)
    err = -ERANGE;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_INIT(psoc4_rtc_init)
{
  struct psoc4_rtc_context_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);

  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         SRSS == addr);

  // 60% accuracy
  pv->freq = DEV_FREQ(32768, 1, 1, 30);

  device_irq_source_init(dev, &pv->irq_ep, 1, &psoc4_rtc_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_pv;

  if (dev_drv_clock_init(dev, &pv->clock_sink, 0, 0, &pv->freq))
    goto unlink_irq;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  uint32_t config;

  cpu_mem_write_32(SRSS + SRSS_WDT_CONTROL_ADDR, 0
                  | SRSS_WDT_CONTROL_ENABLE(0)
                  | SRSS_WDT_CONTROL_ENABLE(2)
                  | SRSS_WDT_CONTROL_RESET(0)
                  | SRSS_WDT_CONTROL_RESET(2)
                  );

  config = 0
    | SRSS_WDT_CONFIG_MODE0(NOTHING)
    | SRSS_WDT_CONFIG_MODE1(NOTHING)
    | SRSS_WDT_CONFIG_MODE2(INT)
    | SRSS_WDT_CONFIG_BITS2(31)
    ;

  SRSS_WDT_CONFIG_LFCLK_SEL_SETVAL(config,
    SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR)));

  cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, config);
  CPU_INTERRUPT_RESTORESTATE;

  dev_request_pqueue_init(&pv->queue);
  pv->base = 0;

  return 0;

 unlink_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(psoc4_rtc_cleanup)
{
  struct psoc4_rtc_context_s *pv = dev->drv_pv;

  if (!dev_request_pqueue_isempty(&pv->queue))
    return -EBUSY;

  dev_request_pqueue_destroy(&pv->queue);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  psoc4_rtc_stop(pv);

  uint32_t config = 0;

  SRSS_WDT_CONFIG_LFCLK_SEL_SETVAL(config,
    SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR)));

  cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, config);

  CPU_INTERRUPT_RESTORESTATE;

  dev_drv_clock_cleanup(dev, &pv->clock_sink);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(psoc4_rtc_drv, 0, "PSoC4 WDT RTC", psoc4_rtc,
               DRIVER_TIMER_METHODS(psoc4_rtc));

DRIVER_REGISTER(psoc4_rtc_drv);
