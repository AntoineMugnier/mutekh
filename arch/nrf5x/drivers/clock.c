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
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/cmu.h>

#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/temp.h>

#define CLOCK_ADDR NRF_PERIPHERAL_ADDR(NRF5X_CLOCK)
#define TEMP_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TEMP)

#define LFRC_CAL defined(CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL)

#if defined(CONFIG_ARCH_NRF51)
# define HFRC_FREQ DEV_FREQ(16000000, 1, 2, 24) // 1%
#else
# define HFRC_FREQ DEV_FREQ(64000000, 1, 7, 24) // 1.5%
#endif
#define LFRC_FREQ DEV_FREQ(32768, 1, 2, 25) // 2%

DRIVER_PV(struct nrf5x_clock_context_s
{
  struct dev_irq_src_s irq_ep[_CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL ? 2 : 1];
  struct dev_clock_src_ep_s src[NRF_CLOCK_SRC_COUNT];

  struct dev_freq_s hfxo_freq;
  struct dev_freq_s lfxo_freq;
  struct dev_freq_s hfclk_freq;
  struct dev_freq_s lfclk_freq;

  uint8_t hf_src;
  uint8_t lf_src;

  uint8_t configid_cur;
  uint8_t configid_next;
  uint8_t configid_app;

  struct kroutine_s configid_updater;

  bool_t lfclk_required;
#if LFRC_CAL
  bool_t cal_en;
  bool_t cal_en_next;
  bool_t cal_done;
  bool_t cal_pending;
  bool_t cal_running;
  bool_t cal_temp_running;

  uint8_t cal_timeout_count;
  int16_t cal_temp_last;
  int16_t temp_cur;
#endif
});

static void nrf5x_clock_configid_refresh(struct device_s *dev);

static bool_t nrf5x_clock_lfclk_is_running(void)
{
  return !!(nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT)
            & NRF_CLOCK_LFCLKSTAT_STATE_RUNNING);
}

static uint_fast8_t nrf5x_clock_lf_src(void)
{
  return nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSRCCOPY);
}

static bool_t nrf5x_clock_lf_is_running(uint_fast8_t type)
{
  uint32_t state = nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT);

  return state == (NRF_CLOCK_LFCLKSTAT_STATE_RUNNING | type);
}

static void nrf5x_clock_lfclk_start(void)
{
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTART);
}

static void nrf5x_clock_lfclk_stop(void)
{
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
}

static bool_t nrf5x_clock_hfxo_is_running(void)
{
  return nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT)
    == (NRF_CLOCK_HF_SRC_XTAL | NRF_CLOCK_HFCLKSTAT_STATE_RUNNING);
}

static bool_t nrf5x_clock_hf_is_running(uint_fast8_t type)
{
  uint32_t state = nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT);

  return state == (NRF_CLOCK_HFCLKSTAT_STATE_RUNNING | type);
}

static uint_fast8_t nrf5x_clock_hf_src(void)
{
  return nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT)
    & NRF_CLOCK_HF_SRC_MASK;
}

static void nrf5x_clock_hfxo_start(void)
{
  dprintk("%s\n", __FUNCTION__);
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTART);
}

static void nrf5x_clock_hfxo_stop(void)
{
  dprintk("%s\n", __FUNCTION__);
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTOP);
}

static DEV_CMU_NODE_INFO(nrf5x_clock_node_info)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;
  static const char *node_name[NRF_CLOCK_NODE_COUNT] = {
    [NRF_CLOCK_SRC_LFCLK] = "LFCLK",
    [NRF_CLOCK_SRC_HFCLK] = "HFCLK",
    [NRF_CLOCK_OSC_LFRC] = "LFRC",
    [NRF_CLOCK_OSC_HFRC] = "HFRC",
    [NRF_CLOCK_OSC_HFXO] = "HFXO",
    [NRF_CLOCK_OSC_LFXO] = "LFXO",
  };
  uint32_t returned = 0;

  if (node_id >= NRF_CLOCK_NODE_COUNT)
    return -EINVAL;

  returned |= DEV_CMU_INFO_NAME;
  returned |= DEV_CMU_INFO_FREQ;
  returned |= DEV_CMU_INFO_ACCURACY;
  returned |= DEV_CMU_INFO_RUNNING;
  info->name = node_name[node_id];

  switch (node_id) {
  case NRF_CLOCK_SRC_LFCLK:
    returned |= DEV_CMU_INFO_SRC;
    returned |= DEV_CMU_INFO_PARENT;
    info->src = &pv->src[NRF_CLOCK_SRC_LFCLK];
    info->running = nrf5x_clock_lfclk_is_running();
    info->freq = pv->lfclk_freq;

    switch (nrf5x_clock_lf_src()) {
    case NRF_CLOCK_LF_SRC_XTAL:
      info->parent_id = NRF_CLOCK_OSC_LFXO;
      break;

    case NRF_CLOCK_LF_SRC_RC:
      info->parent_id = NRF_CLOCK_OSC_LFRC;
      break;

    case NRF_CLOCK_LF_SRC_SYNTH:
      info->parent_id = NRF_CLOCK_SRC_HFCLK;
      break;
    }
    break;

  case NRF_CLOCK_SRC_HFCLK:
    returned |= DEV_CMU_INFO_SRC;
    returned |= DEV_CMU_INFO_PARENT;
    info->freq = pv->hfclk_freq;
    info->src = &pv->src[NRF_CLOCK_SRC_HFCLK];
    info->running = 1;

    switch (nrf5x_clock_hf_src()) {
    case NRF_CLOCK_HF_SRC_RC:
      info->parent_id = NRF_CLOCK_OSC_HFRC;
      break;

    case NRF_CLOCK_HF_SRC_XTAL:
      info->parent_id = NRF_CLOCK_OSC_HFXO;
      break;
    }
    break;

  case NRF_CLOCK_OSC_LFRC:
    info->freq = LFRC_FREQ;
    info->running = nrf5x_clock_lf_is_running(NRF_CLOCK_LF_SRC_RC);
    break;

  case NRF_CLOCK_OSC_LFXO:
    info->freq = pv->lfxo_freq;
    info->running = nrf5x_clock_lf_is_running(NRF_CLOCK_LF_SRC_XTAL);
    break;

  case NRF_CLOCK_OSC_HFRC:
    info->freq = HFRC_FREQ;
    info->running = nrf5x_clock_hf_src() == NRF_CLOCK_HF_SRC_RC;
    break;

  case NRF_CLOCK_OSC_HFXO:
    info->freq = pv->hfxo_freq;
    info->running = nrf5x_clock_hf_src() == NRF_CLOCK_HF_SRC_XTAL;
    break;
  }

  *mask &= returned;

  return 0;
}

#if LFRC_CAL
static void nrf5x_clock_lf_calibrate(struct nrf5x_clock_context_s *pv)
{
  if (pv->cal_en
      && pv->cal_pending
      && !pv->cal_running
      && !pv->cal_temp_running) {
    dwritek("(", 1);
    pv->cal_running = 1;
    pv->cal_temp_running = 1;
    nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
    nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CAL);
  }
}

static DEV_IRQ_SRC_PROCESS(nrf5x_temp_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  if (!nrf_event_check(TEMP_ADDR, NRF_TEMP_DATARDY))
    return;
  nrf_event_clear(TEMP_ADDR, NRF_TEMP_DATARDY);

  dwritek("T", 1);

  pv->cal_temp_running = 0;

  if (pv->cal_running)
    return;

  pv->temp_cur = nrf_reg_get(TEMP_ADDR, NRF_TEMP_TEMP);
  nrf_task_trigger(TEMP_ADDR, NRF_TEMP_STOP);

  if (__ABS(pv->temp_cur - pv->cal_temp_last) > 2 && !pv->cal_pending) {
    pv->cal_pending = 1;

    dwritek("!", 1);

    if (nrf5x_clock_hfxo_is_running())
      nrf5x_clock_lf_calibrate(pv);
    else
      nrf5x_clock_hfxo_start();
  } else {
    pv->cal_timeout_count = 0;
  }
}
#endif

static DEV_IRQ_SRC_PROCESS(nrf5x_clock_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;
  bool_t lf_check = 0;
  bool_t hf_check = 0;

  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED)) {
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED);
    dprintk("%s LFCLK started\n", __FUNCTION__);
    lf_check = 1;
  }

  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED)) {
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED);
    dprintk("%s HFCLK started\n", __FUNCTION__);
    hf_check = 1;
  }

#if LFRC_CAL
  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_DONE)) {
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_DONE);

    dwritek(")", 1);

    pv->cal_temp_last = pv->temp_cur;
    pv->cal_done = pv->cal_en;

    pv->cal_running = 0;
    pv->cal_pending = 0;
    pv->cal_timeout_count = 0;

    hf_check = 1;
    lf_check = 1;
  }
#endif

  if (hf_check) {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    struct dev_clock_notify_s hfclk_notif;
#endif

    if (nrf5x_clock_hfxo_is_running()) {
#if LFRC_CAL
      nrf5x_clock_lf_calibrate(pv);
#endif

      if (pv->hf_src != NRF_CLOCK_HF_SRC_XTAL
#if LFRC_CAL
          && !(pv->cal_pending && pv->cal_en)
#endif
          )
        nrf5x_clock_hfxo_stop();
    }

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    switch (nrf5x_clock_hf_src()) {
    case NRF_CLOCK_HF_SRC_XTAL:
      hfclk_notif.freq = pv->hfxo_freq;
      break;

    case NRF_CLOCK_HF_SRC_RC:
      hfclk_notif.freq = HFRC_FREQ;
      break;
    }

    if (memcmp(&hfclk_notif.freq, &pv->hfclk_freq, sizeof(hfclk_notif.freq))) {
      pv->hfclk_freq = hfclk_notif.freq;
      dev_cmu_src_notify(&pv->src[NRF_CLOCK_SRC_HFCLK], &hfclk_notif);
      lf_check |= (nrf5x_clock_lf_src() == NRF_CLOCK_LF_SRC_SYNTH);
    }
#endif
  }

  if (lf_check) {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    struct dev_clock_notify_s lfclk_notif;
#endif

    if (pv->lfclk_required) {
#if LFRC_CAL
      if (pv->cal_en && !pv->cal_done) {
        pv->cal_done = 0;
        pv->cal_pending = 1;

        dwritek("H", 1);

        if (nrf5x_clock_hfxo_is_running())
          nrf5x_clock_lf_calibrate(pv);
        else
          nrf5x_clock_hfxo_start();

        nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTART);
      }
#endif
      dev_cmu_src_update_async(&pv->src[NRF_CLOCK_SRC_LFCLK], DEV_CLOCK_EP_CLOCK);
    } else {
#if LFRC_CAL
      pv->cal_done = 0;
#endif
      nrf5x_clock_lfclk_stop();
      dev_cmu_src_update_async(&pv->src[NRF_CLOCK_SRC_LFCLK], 0);
    }

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    switch (nrf5x_clock_lf_src()) {
    case NRF_CLOCK_LF_SRC_XTAL:
      lfclk_notif.freq = pv->lfxo_freq;
      break;

    case NRF_CLOCK_LF_SRC_RC:
      lfclk_notif.freq = LFRC_FREQ;
      break;

    case NRF_CLOCK_LF_SRC_SYNTH:
      lfclk_notif.freq.num = 32768;
      lfclk_notif.freq.denom = 1;
      lfclk_notif.freq.acc_m = pv->hfclk_freq.acc_m;
      lfclk_notif.freq.acc_e = pv->hfclk_freq.acc_e;
      break;
    }

    if (memcmp(&lfclk_notif.freq, &pv->lfclk_freq, sizeof(lfclk_notif.freq))) {
      pv->lfclk_freq = lfclk_notif.freq;
      dev_cmu_src_notify(&pv->src[NRF_CLOCK_SRC_LFCLK], &lfclk_notif);
    }
#endif
  }

#if LFRC_CAL
  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_CTTO)) {
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_CTTO);

    dwritek("?", 1);

    pv->cal_timeout_count++;

    if (pv->cal_timeout_count >= 2 && !pv->cal_temp_running) {
      pv->cal_temp_running = 1;
      dwritek("t", 1);
      nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
    }

    if (pv->cal_en)
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTART);
  }
#endif
}

static DEV_CMU_CONFIG_MUX(nrf5x_clock_config_mux)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  switch (node_id) {
  case NRF_CLOCK_SRC_LFCLK:
    switch (parent_id) {
    case NRF_CLOCK_OSC_LFXO:
      pv->lf_src = NRF_CLOCK_LF_SRC_XTAL;
      return 0;

    case NRF_CLOCK_OSC_LFRC:
      pv->lf_src = NRF_CLOCK_LF_SRC_RC;
      return 0;

    case NRF_CLOCK_SRC_HFCLK:
      pv->lf_src = NRF_CLOCK_LF_SRC_SYNTH;
      return 0;

    default:
      return -EINVAL;
    }

  case NRF_CLOCK_SRC_HFCLK:
    switch (parent_id) {
    case NRF_CLOCK_OSC_HFXO:
      pv->hf_src = NRF_CLOCK_HF_SRC_XTAL;
      return 0;

    case NRF_CLOCK_OSC_HFRC:
      pv->hf_src = NRF_CLOCK_HF_SRC_RC;
      return 0;

    default:
      return -EINVAL;
    }
  }

  return -ENOENT;
}

static DEV_CMU_CONFIG_OSC(nrf5x_clock_config_osc)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  switch (node_id) {
  case NRF_CLOCK_OSC_LFXO:
    if (freq->num != 32768 || freq->denom != 1)
      return -EINVAL;
    pv->lfxo_freq = *freq;
    return 0;

  case NRF_CLOCK_OSC_LFRC:
    if (freq->num != 32768 || freq->denom != 1)
      return -EINVAL;
#if LFRC_CAL
    pv->cal_en_next = (freq->acc_e <= 18);
#endif
    return 0;

  case NRF_CLOCK_OSC_HFXO:
    if ((
#if defined(CONFIG_ARCH_NRF51)
         freq->num != 16000000 &&
#endif
         freq->num != 32000000) || freq->denom != 1)
      return -EINVAL;
    pv->hfxo_freq = *freq;
    return 0;

  case NRF_CLOCK_OSC_HFRC:
    if (freq->num != 16000000 || freq->denom != 1)
      return -EINVAL;
    return 0;
  }

  return -ENOENT;
}

static DEV_CMU_ROLLBACK(nrf5x_clock_rollback)
{
#if LFRC_CAL
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  pv->cal_en_next = pv->cal_en;
#endif

  return 0;
}

static DEV_CMU_COMMIT(nrf5x_clock_commit)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  if (pv->lf_src != nrf5x_clock_lf_src() && nrf5x_clock_lfclk_is_running()) {
    nrf5x_clock_lfclk_stop();
    while (nrf5x_clock_lfclk_is_running())
      ;
  }
  nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_LFCLKSRC, pv->lf_src);
#if LFRC_CAL
  pv->cal_en = pv->cal_en_next;
#endif
  if (pv->lfclk_required && !nrf5x_clock_lfclk_is_running())
    nrf5x_clock_lfclk_start();

  if (nrf5x_clock_hf_is_running(NRF_CLOCK_HF_SRC_XTAL)
      && pv->hf_src == NRF_CLOCK_HF_SRC_RC) {
    nrf5x_clock_hfxo_stop();
    while (nrf5x_clock_hfxo_is_running())
      ;
  }

#if defined(CONFIG_ARCH_NRF51)
  nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_XTALFREQ,
              pv->hfxo_freq.num == 16000000
              ? NRF_CLOCK_XTALFREQ_16MHZ
              : NRF_CLOCK_XTALFREQ_32MHZ);
#endif

  if (pv->hf_src == NRF_CLOCK_HF_SRC_XTAL)
    nrf5x_clock_hfxo_start();

  return 0;
}

DRIVER_CMU_CONFIG_OPS_DECLARE(nrf5x_clock);

static DEV_CLOCK_SRC_SETUP(nrf5x_clock_ep_setup)
{
  struct device_s *dev = src->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;
  dev_cmu_node_id_t id = src - pv->src;

  if (id >= NRF_CLOCK_SRC_COUNT)
    return -ENOENT;

  switch (op) {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  case DEV_CLOCK_SRC_SETUP_NOTIFY:
  case DEV_CLOCK_SRC_SETUP_NONOTIFY:
    return 0;
#endif

  case DEV_CLOCK_SRC_SETUP_SCALER:
    return -ENOTSUP;

#ifdef CONFIG_DEVICE_CLOCK_THROTTLE
  case DEV_CLOCK_SRC_SETUP_THROTTLE:
    dprintk("%s src %d throttle %d->%d\n",
            __FUNCTION__,
            id,
            param->throttle.configid_old,
            param->throttle.configid_new);
    nrf5x_clock_configid_refresh(dev);
    return 0;
#endif

  case DEV_CLOCK_SRC_SETUP_LINK:
    if (param->sink->flags & DEV_CLOCK_EP_GATING_SYNC)
      return -ENOTSUP;
    return 0;

  case DEV_CLOCK_SRC_SETUP_UNLINK:
    return 0;

  case DEV_CLOCK_SRC_SETUP_GATES:
    switch (id) {
    case NRF_CLOCK_SRC_LFCLK:
      if (!!(param->flags & DEV_CLOCK_EP_CLOCK) == nrf5x_clock_lfclk_is_running()) {
        dev_cmu_src_update_sync(src, param->flags);
        return 0;
      } else {
        pv->lfclk_required = !!(param->flags & DEV_CLOCK_EP_CLOCK);
        nrf5x_clock_lfclk_start();
        return -EAGAIN;
      }

    case NRF_CLOCK_SRC_HFCLK:
      dev_cmu_src_update_sync(src, param->flags);
      return 0;
    }
    break;
  }

  return 0;
}

static DEV_USE(nrf5x_clock_use)
{
  switch (op) {
#ifdef CONFIG_DEVICE_CLOCK_THROTTLE
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct nrf5x_clock_context_s *pv = dev->drv_pv;

    if (pv->configid_cur != pv->configid_next)
      kroutine_exec(&pv->configid_updater);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

static void nrf5x_clock_configid_refresh(struct device_s *dev)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  uint_fast8_t configid
    = __MAX((uint_fast8_t)pv->configid_app,
            __MAX((uint_fast8_t)pv->src[NRF_CLOCK_SRC_LFCLK].configid_min,
                  (uint_fast8_t)pv->src[NRF_CLOCK_SRC_HFCLK].configid_min));

  dprintk("%s %d %d %d: %d->%d\n", __FUNCTION__,
          (uint_fast8_t)pv->configid_app,
          (uint_fast8_t)pv->src[NRF_CLOCK_SRC_LFCLK].configid_min,
          (uint_fast8_t)pv->src[NRF_CLOCK_SRC_HFCLK].configid_min,
          pv->configid_cur, configid);

  pv->configid_next = configid;

  if (pv->configid_cur == pv->configid_next)
    return;

  if (pv->configid_cur < pv->configid_next) {
    pv->configid_cur = pv->configid_next;
    dev_cmu_configid_set(dev, &nrf5x_clock_config_ops, pv->configid_next);
  } else {
    device_sleep_schedule(dev);
  }
}

static DEV_CMU_APP_CONFIGID_SET(nrf5x_clock_app_configid_set)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);
  
  pv->configid_app = config_id;
  nrf5x_clock_configid_refresh(dev);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static KROUTINE_EXEC(nrf5x_clock_configid_update)
{
  struct nrf5x_clock_context_s *pv = KROUTINE_CONTAINER(kr, *pv, configid_updater);
  struct device_s *dev = pv->src[0].dev;

  LOCK_SPIN_IRQ(&dev->lock);

  dprintk("%s %d->%d\n", __FUNCTION__,
          pv->configid_cur, pv->configid_next);

  pv->configid_cur = pv->configid_next;
  dev_cmu_configid_set(dev, &nrf5x_clock_config_ops, pv->configid_next);

  LOCK_RELEASE_IRQ(&dev->lock);
}

const struct driver_s nrf5x_clock_drv;

static DEV_INIT(nrf5x_clock_init)
{
  struct nrf5x_clock_context_s *pv;
  uint8_t i;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_CLOCK) == addr);
#if LFRC_CAL
  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_TEMP) == addr);
#endif

  pv->lfclk_freq = LFRC_FREQ;
  pv->hfclk_freq = HFRC_FREQ;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_clock_irq);
#if LFRC_CAL
  device_irq_source_init(dev, pv->irq_ep + 1, 1, &nrf5x_temp_irq);
#endif

  err = device_irq_source_link(dev, pv->irq_ep,
                               _CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL ? 2 : 1,
                               -1);
  if (err)
    goto free_pv;

#if LFRC_CAL && defined(CONFIG_ARCH_NRF52)
  // PAN 36: CLOCK: Some registers are not reset when expected
  nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_CTTO);
  nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_DONE);
#endif
  nrf_it_disable_mask(CLOCK_ADDR, -1);
  nrf_it_enable_mask(CLOCK_ADDR, 0
#if LFRC_CAL
                     | bit(NRF_CLOCK_CTTO)
                     | bit(NRF_CLOCK_DONE)
#endif
                     | bit(NRF_CLOCK_HFCLKSTARTED)
                     | bit(NRF_CLOCK_LFCLKSTARTED)
                     );

#if LFRC_CAL
  nrf_it_disable_mask(TEMP_ADDR, -1);
  nrf_it_enable(TEMP_ADDR, NRF_TEMP_DATARDY);

  nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_CTIV, 4);
#endif

  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTOP);

  kroutine_init_deferred(&pv->configid_updater, &nrf5x_clock_configid_update);

  for (i = 0; i < NRF_CLOCK_SRC_COUNT; i++)
    dev_clock_source_init(dev, &pv->src[i], &nrf5x_clock_ep_setup);

  err = dev_cmu_init(dev, &nrf5x_clock_config_ops);
  if (err)
    goto free_pv;

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(nrf5x_clock_cleanup)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);

  device_irq_source_unlink(dev, pv->irq_ep,
                           _CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL ? 2 : 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_clock_drv, DRIVER_FLAGS_EARLY_INIT | DRIVER_FLAGS_NO_DEPEND, "nRF5x clock"
#if LFRC_CAL
               "/Cal"
#endif
               , nrf5x_clock,
               DRIVER_CMU_METHODS(nrf5x_clock));

DRIVER_REGISTER(nrf5x_clock_drv);
