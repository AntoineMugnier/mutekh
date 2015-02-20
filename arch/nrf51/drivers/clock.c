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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

//#define dprintk printk
#ifndef dprintk
# define dprintk(...) do{}while(0)
#endif

#if defined(CONFIG_DRIVER_NRF51_CLOCK)
# include <device/device.h>
# include <device/resources.h>
# include <device/driver.h>
# include <device/irq.h>
# include <device/class/clock.h>
#endif

#include <arch/nrf51/clock.h>

#define CLOCK_ADDR NRF_PERIPHERAL_ADDR(NRF51_CLOCK)

#if defined(CONFIG_DRIVER_NRF51_CLOCK)

#define ACC_100PPM DEV_FREQ_ACC(4, 17)
#define ACC_100PPM DEV_FREQ_ACC(4, 17)
#define ACC_LF_RC DEV_FREQ_ACC(0, 9) // 256ppm
#define ACC_HF_RC DEV_FREQ_ACC(7, 20) // 983ppm

enum nrf51_clock_mode_e
{
  MODE_OFF,
  MODE_XTAL,
  MODE_RC,
  MODE_RC_CALIBRATED,
};

enum nrf51_calib_state_e
{
  CALIB_NONE,
  CALIB_PENDING,
  CALIB_RUNNING,
  CALIB_DONE,
};

struct nrf51_clock_context_s
{
  struct dev_irq_ep_s irq_ep[1];

  struct dev_clock_src_ep_s src[NRF51_CLOCK_EP_COUNT];

  struct dev_freq_accuracy_s hfxo_acc;
  struct dev_freq_accuracy_s lfxo_acc;

#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
  bool_t lf_calib_done;
  enum nrf51_calib_state_e lf_calib_state;
#endif
  enum nrf51_clock_mode_e lf_mode, hf_mode;
};

static void hf_clock_mode_update(struct nrf51_clock_context_s *pv, bool_t synchronous);

static DEV_CLOCK_NODE_INFO(nrf51_clock_node_info)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_clock_context_s *pv = dev->drv_pv;

  if (node_id >= NRF51_CLOCK_EP_COUNT)
    return -ENOENT;

  switch (node_id) {
  case NRF51_CLOCK_LF_CALIBRATED:
  case NRF51_CLOCK_LF:
    info->freq.num = 32768;
    info->freq.denom = 1;
    break;

#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
  case NRF51_CLOCK_HF_PRECISE:
#endif
  case NRF51_CLOCK_HF:
    info->freq.num = 32000000;
    info->freq.denom = 1;
    break;
  }

  *mask &= ~DEV_CLOCK_INFO_SINK;
  info->src = NULL;
  info->parent_id = -1;

  switch (node_id) {
# if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
  case NRF51_CLOCK_LF:
    info->name = "LFCLK (RC)";
    info->src = &pv->src[NRF51_CLOCK_LF];
    info->acc = ACC_LF_RC;
    info->running = nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKRUN);
    break;

  case NRF51_CLOCK_LF_CALIBRATED:
    info->name = "LFCLK (RC, Calibrated)";
    info->src = &pv->src[NRF51_CLOCK_LF_CALIBRATED];
    info->acc = ACC_LF_RC;
    info->running = nrf_it_is_enabled(CLOCK_ADDR, NRF51_CLOCK_CTTO);
    break;

#else
  case NRF51_CLOCK_LF:
  case NRF51_CLOCK_LF_CALIBRATED:
    info->name = "LFCLK (XO)";
    info->src = &pv->src[NRF51_CLOCK_LF_CALIBRATED];
    info->acc = ACC_LF_RC;
    info->running = !!(nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTAT)
                       & NRF51_CLOCK_LFCLKSTAT_STATE_RUNNING);
    break;
#endif

# if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
  case NRF51_CLOCK_HF_PRECISE:
    info->running = (nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTAT) & NRF51_CLOCK_HF_SRC_MASK)
      == NRF51_CLOCK_HF_SRC_XTAL;
    info->acc = pv->hfxo_acc;
    info->name = "HFCLK (calibrated)";
    info->src = &pv->src[NRF51_CLOCK_HF_PRECISE];
    break;
#endif

  case NRF51_CLOCK_HF:
    switch (nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTAT) & NRF51_CLOCK_HF_SRC_MASK) {
    case NRF51_CLOCK_HF_SRC_RC:
      info->acc = ACC_HF_RC;
      break;
    case NRF51_CLOCK_HF_SRC_XTAL:
      info->acc = pv->hfxo_acc;
      break;
    }
    info->name = "HFCLK (default)";
    info->src = &pv->src[NRF51_CLOCK_HF];
    info->running = 1;
    break;
  }

  return 0;
}

static DEV_CLOCK_CONFIG_NODE(nrf51_clock_config_node)
{
  return -ENOTSUP;
}

static DEV_CLOCK_COMMIT(nrf51_clock_commit)
{
  return -ENOTSUP;
}

static DEV_CLOCK_ROLLBACK(nrf51_clock_rollback)
{
  return -ENOTSUP;
}

static void clock_lf_notify_changed(struct nrf51_clock_context_s *pv)
{
  struct dev_freq_accuracy_s acc;
  struct dev_freq_s freq = { 32768, 1 };
  enum nrf51_clock_mode_e mode = MODE_OFF;

  if (nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTAT) & NRF51_CLOCK_LFCLKSTAT_STATE_RUNNING
      && nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKRUN)) {
    switch (nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTAT) & NRF51_CLOCK_LF_SRC_MASK) {
    case NRF51_CLOCK_LF_SRC_RC:
      acc = ACC_LF_RC;
#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
      if (pv->lf_calib_done)
        mode = MODE_RC_CALIBRATED;
      else
        mode = MODE_RC;
#else
      mode = MODE_RC;
#endif
      break;

    case NRF51_CLOCK_LF_SRC_XTAL:
      acc = pv->lfxo_acc;
      mode = MODE_XTAL;
      break;

    case NRF51_CLOCK_LF_SRC_SYNTH:
      acc = pv->hfxo_acc;
      mode = MODE_XTAL;
      break;
    }
  }

  if (pv->lf_mode == mode)
    return;

  pv->lf_mode = mode;

  switch (pv->lf_mode) {
  case MODE_XTAL:
  case MODE_RC_CALIBRATED:
    pv->src[NRF51_CLOCK_LF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
    pv->src[NRF51_CLOCK_LF_CALIBRATED].flags |= DEV_CLOCK_SRC_EP_RUNNING;
    break;

  case MODE_RC:
    pv->src[NRF51_CLOCK_LF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
    pv->src[NRF51_CLOCK_LF_CALIBRATED].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
    break;

  case MODE_OFF:
    pv->src[NRF51_CLOCK_LF].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
    pv->src[NRF51_CLOCK_LF_CALIBRATED].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
    break;
  }

  dprintk("%c", "oxrc"[pv->lf_mode]);

  if (pv->src[NRF51_CLOCK_LF_CALIBRATED].flags & DEV_CLOCK_SRC_EP_NOTIFY)
    dev_clock_src_changed(NULL, pv->src + NRF51_CLOCK_LF_CALIBRATED, &freq, &acc);

  if (pv->src[NRF51_CLOCK_LF].flags & DEV_CLOCK_SRC_EP_NOTIFY)
    dev_clock_src_changed(NULL, pv->src + NRF51_CLOCK_LF, &freq, &acc);
}

static void clock_hf_notify_changed(struct nrf51_clock_context_s *pv)
{
  struct dev_freq_accuracy_s acc = ACC_HF_RC;
  struct dev_freq_s freq = { 32000000, 1 };
  enum nrf51_clock_mode_e mode = MODE_RC;

#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
  if ((nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTAT)
       & NRF51_CLOCK_HFCLKSTAT_STATE_RUNNING)
      && nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_HFCLKRUN))
    mode = MODE_XTAL;
#endif

  if (pv->hf_mode == mode)
    return;

  pv->hf_mode = mode;

  switch (pv->hf_mode) {
  case MODE_XTAL:
    acc = pv->hfxo_acc;
    pv->src[NRF51_CLOCK_HF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
    pv->src[NRF51_CLOCK_HF_PRECISE].flags |= DEV_CLOCK_SRC_EP_RUNNING;
#endif
    break;

  case MODE_RC_CALIBRATED:
  case MODE_RC:
    pv->src[NRF51_CLOCK_HF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
    pv->src[NRF51_CLOCK_HF_PRECISE].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
#endif
    break;

  case MODE_OFF:
    pv->src[NRF51_CLOCK_HF].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
    pv->src[NRF51_CLOCK_HF_PRECISE].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
#endif
    break;
  }

  dprintk("%c", "OXRC"[pv->hf_mode]);

  if (pv->src[NRF51_CLOCK_HF_PRECISE].flags & DEV_CLOCK_SRC_EP_NOTIFY)
    dev_clock_src_changed(NULL, pv->src + NRF51_CLOCK_HF_PRECISE, &freq, &acc);

  if (pv->src[NRF51_CLOCK_HF].flags & DEV_CLOCK_SRC_EP_NOTIFY)
    dev_clock_src_changed(NULL, pv->src + NRF51_CLOCK_HF, &freq, &acc);
}

static DEV_IRQ_EP_PROCESS(nrf51_clock_irq)
{
  struct device_s *dev = ep->dev;
  struct nrf51_clock_context_s *pv = dev->drv_pv;

  if (nrf_event_check(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTARTED))
    nrf_event_clear(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTARTED);

  if (nrf_event_check(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTARTED))
    nrf_event_clear(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTARTED);

  clock_lf_notify_changed(pv);
  clock_hf_notify_changed(pv);

#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
  if (nrf_event_check(CLOCK_ADDR, NRF51_CLOCK_CTTO)) {
    nrf_event_clear(CLOCK_ADDR, NRF51_CLOCK_CTTO);

    if (pv->lf_mode != MODE_OFF) {
      nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_CTIV, 2.5 * 4);
      nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_CTSTART);

      switch (pv->lf_calib_state) {
      case CALIB_DONE:
        dprintk("?");
        pv->lf_calib_state = CALIB_PENDING;
        break;
      case CALIB_PENDING:
        dprintk("!");
        hf_clock_mode_update(pv, 0);
        break;
      default:
        break;
      }
    }
  }

  if (pv->lf_calib_state == CALIB_PENDING
      && pv->lf_mode != MODE_OFF
      && pv->hf_mode == MODE_XTAL) {
    dprintk(":");

    pv->lf_calib_state = CALIB_RUNNING;
    nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_CAL);
  }

  if (nrf_event_check(CLOCK_ADDR, NRF51_CLOCK_DONE)) {
    nrf_event_clear(CLOCK_ADDR, NRF51_CLOCK_DONE);

    dprintk(".");

    pv->lf_calib_state = CALIB_DONE;
    pv->lf_calib_done = 1;

    hf_clock_mode_update(pv, 0);
  }
#endif
}

static void lf_clock_mode_update(struct nrf51_clock_context_s *pv, bool_t synchronous)
{
  enum nrf51_clock_mode_e expected_mode = MODE_OFF;

  if (pv->src[NRF51_CLOCK_LF_CALIBRATED].use_count) {
#if defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
    expected_mode = MODE_XTAL;
#else
    expected_mode = MODE_RC_CALIBRATED;
#endif
  } else if (pv->src[NRF51_CLOCK_LF].use_count)
    expected_mode = MODE_RC;

  switch (expected_mode) {
  case MODE_OFF:
    synchronous = 1;
    if (nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKRUN)) {
#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
      pv->lf_calib_done = 0;
      pv->lf_calib_state = CALIB_NONE;
      nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_CTSTOP);
#endif
      nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTOP);
    }
    break;

  default:
    if (!nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_LFCLKRUN)) {
#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
      pv->lf_calib_state = expected_mode == MODE_RC_CALIBRATED ? CALIB_PENDING : CALIB_NONE;
      nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_CTIV, 1);
      nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_CTSTART);
#endif
      nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTOP);
      nrf_event_clear(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTARTED);
      nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTART);
      // IRQ will enable calibration and/or notify
    }
    break;
  }

  if (synchronous) {
    dprintk("{%c", "oxrc"[expected_mode]);
    while (pv->lf_mode != expected_mode) {
      if (!cpu_is_interruptible())
        nrf51_clock_irq(pv->irq_ep, NULL);
      order_compiler_mem();
    }
    dprintk("}");
  }
}

static void hf_clock_mode_update(struct nrf51_clock_context_s *pv, bool_t synchronous)
{
# if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
  enum nrf51_clock_mode_e expected_mode = MODE_RC;
  if (pv->src[NRF51_CLOCK_HF_PRECISE].use_count)
    expected_mode = MODE_XTAL;

#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
  if (pv->lf_calib_state != CALIB_DONE)
    expected_mode = MODE_XTAL;
#endif

  if (expected_mode == MODE_XTAL && !nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_HFCLKRUN)) {
    nrf_event_clear(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTARTED);
    nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTART);
    // IRQ will enable calibration and/or notify
  } else if (expected_mode != MODE_XTAL && nrf_reg_get(CLOCK_ADDR, NRF51_CLOCK_HFCLKRUN)) {
    nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTOP);
    synchronous = 1;
  }

  if (synchronous) {
    dprintk("{%c", "OXRC"[expected_mode]);
    while (pv->hf_mode != expected_mode) {
      if (!cpu_is_interruptible())
        nrf51_clock_irq(pv->irq_ep, NULL);
      order_compiler_mem();
    }
    dprintk("}");
  }
#endif
}

static DEV_CLOCK_SRC_USE(nrf51_clock_ep_use)
{
  struct device_s *dev = src->dev;
  struct nrf51_clock_context_s *pv = dev->drv_pv;
  dev_clock_node_id_t id = src - pv->src;

  if (id > NRF51_CLOCK_EP_COUNT)
    return -EINVAL;

  switch (action) {
  case DEV_CLOCK_SRC_USE_HOLD:
  case DEV_CLOCK_SRC_USE_RELEASE:
    switch (id) {
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
    case NRF51_CLOCK_HF_PRECISE:
#endif
    case NRF51_CLOCK_HF:
      hf_clock_mode_update(pv, synchronous);
      break;

    case NRF51_CLOCK_LF:
    case NRF51_CLOCK_LF_CALIBRATED:
      lf_clock_mode_update(pv, synchronous);
      break;
    }
    break;

  case DEV_CLOCK_SRC_USE_NOTIFY:
  case DEV_CLOCK_SRC_USE_IGNORE:
    break;
  }

  return 0;
}

static DEV_INIT(nrf51_clock_init);
static DEV_CLEANUP(nrf51_clock_cleanup);
#define nrf51_clock_use dev_use_generic

DRIVER_DECLARE(nrf51_clock_drv, "nRF51 clock"
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ == 16
               " hf=16MHz"
#elif CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ == 32
               " hf=32MHz"
#else
               " hf=RC"
#endif
#define nrf51_clock_use dev_use_generic

#if defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
               " lf=XO"
#else
               " lf=RC"
#endif
               , nrf51_clock,
               DRIVER_CLOCK_METHODS(nrf51_clock));

DRIVER_REGISTER(nrf51_clock_drv);

static DEV_INIT(nrf51_clock_init)
{
  struct nrf51_clock_context_s *pv;
  uint8_t i;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF51_CLOCK) == addr);

  pv->lfxo_acc = ACC_100PPM;
  pv->hfxo_acc = ACC_100PPM;
  pv->lf_mode = MODE_OFF;
  pv->hf_mode = MODE_RC;

  device_irq_source_init(dev, pv->irq_ep, 1,
                         &nrf51_clock_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);
  nrf_it_enable_mask(CLOCK_ADDR, 0
#if !defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
                     | (1 << NRF51_CLOCK_CTTO)
                     | (1 << NRF51_CLOCK_DONE)
#endif
                     | (1 << NRF51_CLOCK_HFCLKSTARTED)
                     | (1 << NRF51_CLOCK_LFCLKSTARTED)
                     );

  for (i = 0; i < NRF51_CLOCK_EP_COUNT; i++)
    dev_clock_source_init(dev, &pv->src[i], &nrf51_clock_ep_use);

  dev->drv = &nrf51_clock_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(nrf51_clock_cleanup)
{
  struct nrf51_clock_context_s *pv = dev->drv_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);
}
#endif

void arch_nrf51_clock_init()
{
  // Always configure this, it will be useful for driver as well.
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ == 16
  nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_XTALFREQ, NRF51_CLOCK_XTALFREQ_16MHZ);
#else
  nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_XTALFREQ, NRF51_CLOCK_XTALFREQ_32MHZ);
#endif

#if defined(CONFIG_DRIVER_NRF51_CLOCK_LFCLK_XOSC)
  nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_LFCLKSRC, NRF51_CLOCK_LF_SRC_XTAL);
#elif !defined(CONFIG_DRIVER_NRF51_CLOCK)
  // As long as we have an external quartz and no driver, we dont care
  // about power saving. We can create a 32k clock with HFCLK
  nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_LFCLKSRC, NRF51_CLOCK_LF_SRC_SYNTH);
#else
  // If we are here, we have no external oscillators at all in the
  // platform.
  nrf_reg_set(CLOCK_ADDR, NRF51_CLOCK_LFCLKSRC, NRF51_CLOCK_LF_SRC_RC);
#endif

#if !defined(CONFIG_DRIVER_NRF51_CLOCK)
# if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
  nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF51_CLOCK_HFCLKSTARTED);
# endif

  nrf_task_trigger(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF51_CLOCK_LFCLKSTARTED);
#endif
}
