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
#include <mutek/startup.h>

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#if defined(CONFIG_DRIVER_NRF5X_CLOCK)
# include <device/device.h>
# include <device/resources.h>
# include <device/driver.h>
# include <device/irq.h>
# include <device/class/clock.h>
#endif

#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/temp.h>

#define CLOCK_ADDR NRF_PERIPHERAL_ADDR(NRF5X_CLOCK)
#define TEMP_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TEMP)

#if defined(CONFIG_DRIVER_NRF5X_CLOCK)

//#define GPIO_DEBUG

#if defined(GPIO_DEBUG)

# include <arch/nrf5x/gpio.h>
# include <arch/nrf5x/gpiote.h>
# include <arch/nrf5x/ppi.h>
# include <arch/nrf5x/rtc.h>

# define I_HFCLOCK_RUN (1 << 24)
# define I_HFCLOCK_REQ (1 << 30)
# define I_LFCLK       0//(1 << 19)
# define I_CAL         0//(1 << 20)
#define GPIOTE_ADDR NRF_PERIPHERAL_ADDR(NRF5X_GPIOTE)

static void debug_init(void)
{
  uint32_t gpios = 0
    | I_HFCLOCK_RUN
    | I_HFCLOCK_REQ
    | I_LFCLK
    | I_CAL;

  while (gpios) {
    uint8_t i = __builtin_ctz(gpios);
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(i), 0
                | NRF_GPIO_PIN_CNF_DIR_OUTPUT
                | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
                | NRF_GPIO_PIN_CNF_DRIVE_S0S1
                );

    gpios &= ~(1 << i);
  }

# if I_LFCLK
  nrf_ppi_setup(15,
                  NRF_PERIPHERAL_ADDR(NRF5X_RTC0), NRF_RTC_TICK,
                  GPIOTE_ADDR, NRF_GPIOTE_OUT(1));

  nrf_evt_enable(NRF_PERIPHERAL_ADDR(NRF5X_RTC0), NRF_RTC_TICK);

  nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(1), 0
              | NRF_GPIOTE_CONFIG_MODE_TASK
              | NRF_GPIOTE_CONFIG_PSEL(19)
              | NRF_GPIOTE_CONFIG_OUTINIT_LOW
              | NRF_GPIOTE_CONFIG_POLARITY_TOGGLE);

  nrf_ppi_enable(15);
# endif
}

static inline void gpio(uint32_t mask, uint32_t value)
{
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUT, 0
              | (nrf_reg_get(NRF5X_GPIO_ADDR, NRF_GPIO_OUT) & (~mask))
              | (value)
              );
}

#else

# define gpio(a, b) do{}while(0)
# define debug_init()

#endif

#define ACC_20PPM DEV_FREQ_ACC(2, 15)
#define ACC_100PPM DEV_FREQ_ACC(4, 17)
#define ACC_LF_RC DEV_FREQ_ACC(2, 25) // 2%
#define ACC_LF_RC_CALIBRATED DEV_FREQ_ACC(0, 19) // 256ppm
#define ACC_HF_RC DEV_FREQ_ACC(7, 20) // 983ppm

enum nrf5x_clock_mode_e
{
  MODE_OFF,
  MODE_RC,
  MODE_RC_PRECISE,
  MODE_XTAL,
};

enum nrf5x_calib_state_e
{
  CALIB_NONE,
  CALIB_NOTYET,
  CALIB_RUNNING,
  CALIB_DONE,
  CALIB_AGAIN_PENDING,
  CALIB_AGAIN_CHECKING,
  CALIB_AGAIN_RUNNING,
};

struct nrf5x_clock_context_s
{
  struct dev_irq_src_s irq_ep[_CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? 1 : 2];

  struct dev_clock_src_ep_s src[NRF_CLOCK_EP_COUNT];

  struct dev_freq_accuracy_s hfxo_acc;
  struct dev_freq_accuracy_s lfxo_acc;

  struct dev_freq_accuracy_s hf_acc;
  struct dev_freq_accuracy_s lf_acc;

#if defined(CONFIG_DRIVER_NRF5X_CLOCK_USE)
  uint8_t force[NRF_CLOCK_EP_COUNT];
#endif
  bool_t notifying;
  uint8_t changed_sources;

  struct kroutine_s state_checker;

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  int32_t lf_calib_temp;
  enum nrf5x_calib_state_e lf_calib_state;
#endif
  enum nrf5x_clock_mode_e lf_mode, hf_mode;
};

static void hf_clock_mode_update(struct nrf5x_clock_context_s *pv);

static DEV_CLOCK_NODE_INFO(nrf5x_clock_node_info)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  if (node_id >= NRF_CLOCK_EP_COUNT)
    return -ENOENT;

  switch (node_id) {
  case NRF_CLOCK_LF_PRECISE:
  case NRF_CLOCK_LF:
    info->freq.num = 32768;
    info->freq.denom = 1;
    break;

#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  case NRF_CLOCK_HF_PRECISE:
#endif
  case NRF_CLOCK_HF:
    info->freq.num = 16000000;
    info->freq.denom = 1;
    break;
  }

  *mask &= ~DEV_CLOCK_INFO_SINK;
  info->src = NULL;
  info->parent_id = -1;

  switch (node_id) {
# if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  case NRF_CLOCK_LF:
    info->name = "LFCLK (RC)";
    info->src = &pv->src[NRF_CLOCK_LF];
    info->acc = ACC_LF_RC;
    info->running = nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN);
    break;

  case NRF_CLOCK_LF_PRECISE:
    info->name = "LFCLK (RC, Precise)";
    info->src = &pv->src[NRF_CLOCK_LF_PRECISE];
    info->acc = ACC_LF_RC_CALIBRATED;
    info->running = nrf_it_is_enabled(CLOCK_ADDR, NRF_CLOCK_DONE);
    break;

#else
  case NRF_CLOCK_LF:
  case NRF_CLOCK_LF_PRECISE:
    info->name = "LFCLK (XO)";
    info->src = &pv->src[NRF_CLOCK_LF_PRECISE];
    info->acc = pv->lfxo_acc;
    info->running = !!(nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT)
                       & NRF_CLOCK_LFCLKSTAT_STATE_RUNNING);
    break;
#endif

# if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  case NRF_CLOCK_HF_PRECISE:
    info->running = (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT) & NRF_CLOCK_HF_SRC_MASK)
      == NRF_CLOCK_HF_SRC_XTAL;
    info->acc = pv->hfxo_acc;
    info->name = "HFCLK (precise)";
    info->src = &pv->src[NRF_CLOCK_HF_PRECISE];
    break;
#endif

  case NRF_CLOCK_HF:
    switch (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT) & NRF_CLOCK_HF_SRC_MASK) {
    case NRF_CLOCK_HF_SRC_RC:
      info->acc = ACC_HF_RC;
      break;
    case NRF_CLOCK_HF_SRC_XTAL:
      info->acc = pv->hfxo_acc;
      break;
    }
    info->name = "HFCLK (default)";
    info->src = &pv->src[NRF_CLOCK_HF];
    info->running = 1;
    break;
  }

  return 0;
}

static DEV_CLOCK_CONFIG_ROUTE(nrf5x_clock_config_route)
{
  return -ENOTSUP;
}

static DEV_CLOCK_CONFIG_OSCILLATOR(nrf5x_clock_config_oscillator)
{
  return -ENOTSUP;
}

static DEV_CLOCK_COMMIT(nrf5x_clock_commit)
{
  return -ENOTSUP;
}

static DEV_CLOCK_ROLLBACK(nrf5x_clock_rollback)
{
  return -ENOTSUP;
}

static enum nrf5x_clock_mode_e clock_lf_expected_mode_get(struct nrf5x_clock_context_s *pv)
{
  if (pv->src[NRF_CLOCK_LF_PRECISE].use_count
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_USE)
      || pv->force[NRF_CLOCK_LF_PRECISE]
#endif
      ) {
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
    return MODE_XTAL;
#else
    return MODE_RC_PRECISE;
#endif
  }

  if (pv->src[NRF_CLOCK_LF].use_count
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_USE)
             || pv->force[NRF_CLOCK_LF]
#endif
             )
    return MODE_RC;

  return MODE_OFF;
}

static enum nrf5x_clock_mode_e clock_hf_expected_mode_get(struct nrf5x_clock_context_s *pv)
{
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  if (pv->src[NRF_CLOCK_HF_PRECISE].use_count
# if defined(CONFIG_DRIVER_NRF5X_CLOCK_USE)
      || pv->force[NRF_CLOCK_HF_PRECISE]
# endif
      )
    return MODE_XTAL;

# if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  switch (pv->lf_calib_state) {
  case CALIB_DONE:
  case CALIB_NONE:
    return MODE_RC;

  case CALIB_AGAIN_PENDING:
  case CALIB_NOTYET:
  case CALIB_AGAIN_CHECKING:
  case CALIB_AGAIN_RUNNING:
  case CALIB_RUNNING:
    return MODE_XTAL;
  }
# endif
#endif

  return MODE_RC;
}

static enum nrf5x_clock_mode_e clock_lf_mode_get(struct nrf5x_clock_context_s *pv)
{
  if (!(nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT) & NRF_CLOCK_LFCLKSTAT_STATE_RUNNING)
      || !nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN))
    return MODE_OFF;

  switch (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT) & NRF_CLOCK_LF_SRC_MASK) {
  case NRF_CLOCK_LF_SRC_RC:
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
    switch (pv->lf_calib_state) {
    case CALIB_AGAIN_RUNNING:
    case CALIB_AGAIN_CHECKING:
    case CALIB_DONE:
    case CALIB_AGAIN_PENDING:
      return MODE_RC_PRECISE;

    case CALIB_NONE:
    case CALIB_RUNNING:
    case CALIB_NOTYET:
      return MODE_RC;
    }
#endif
    return MODE_RC;

  case NRF_CLOCK_LF_SRC_XTAL:
  case NRF_CLOCK_LF_SRC_SYNTH:
    return MODE_XTAL;
  }

  return MODE_OFF;
}

static enum nrf5x_clock_mode_e clock_hf_mode_get(struct nrf5x_clock_context_s *pv)
{
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  if ((nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT) & NRF_CLOCK_HFCLKSTAT_STATE_RUNNING)
      && nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKRUN))
    return MODE_XTAL;
#endif

  return MODE_RC;
}

static void clock_lf_status_update(struct nrf5x_clock_context_s *pv)
{
  enum nrf5x_clock_mode_e mode = MODE_OFF;

  dprintk("d");

  if (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT) & NRF_CLOCK_LFCLKSTAT_STATE_RUNNING
      && nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN)) {
    switch (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT) & NRF_CLOCK_LF_SRC_MASK) {
    case NRF_CLOCK_LF_SRC_RC:
      pv->lf_acc = ACC_LF_RC;
      mode = MODE_RC;
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
      switch (pv->lf_calib_state) {
      case CALIB_AGAIN_RUNNING:
      case CALIB_AGAIN_CHECKING:
      case CALIB_DONE:
        pv->lf_acc = ACC_LF_RC_CALIBRATED;
        mode = MODE_RC_PRECISE;
      case CALIB_NONE:
      case CALIB_RUNNING:
        break;

      case CALIB_AGAIN_PENDING:
        pv->lf_acc = ACC_LF_RC_CALIBRATED;
        mode = MODE_RC_PRECISE;
        if (pv->hf_mode == MODE_XTAL) {
          dprintk("~");
          nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
          gpio(I_CAL, I_CAL);
        }
        break;

      case CALIB_NOTYET:
        if (pv->hf_mode == MODE_XTAL) {
          dprintk("*");
          nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
          nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CAL);
          pv->lf_calib_state = CALIB_RUNNING;
          gpio(I_CAL, I_CAL);
        }
        break;
      }
#endif
      break;

    case NRF_CLOCK_LF_SRC_XTAL:
      pv->lf_acc = pv->lfxo_acc;
      mode = MODE_XTAL;
      break;

    case NRF_CLOCK_LF_SRC_SYNTH:
      pv->lf_acc = pv->hfxo_acc;
      mode = MODE_XTAL;
      break;
    }
  }

  dprintk("%c", "orcx"[mode]);
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  dprintk("%d", pv->lf_calib_state);
#endif

  if (pv->lf_mode == mode)
    return;

  pv->lf_mode = mode;

  switch (pv->lf_mode) {
  case MODE_XTAL:
  case MODE_RC_PRECISE:
    pv->src[NRF_CLOCK_LF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
    pv->src[NRF_CLOCK_LF_PRECISE].flags |= DEV_CLOCK_SRC_EP_RUNNING;
    break;

  case MODE_RC:
    pv->src[NRF_CLOCK_LF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
    pv->src[NRF_CLOCK_LF_PRECISE].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
    break;

  case MODE_OFF:
    pv->src[NRF_CLOCK_LF].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
    pv->src[NRF_CLOCK_LF_PRECISE].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
    break;
  }

  pv->changed_sources |= (1 << NRF_CLOCK_LF);
}

static void clock_hf_status_update(struct nrf5x_clock_context_s *pv)
{
  enum nrf5x_clock_mode_e mode = MODE_RC;

#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  if ((nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT)
       & NRF_CLOCK_HFCLKSTAT_STATE_RUNNING)
      && nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKRUN))
    mode = MODE_XTAL;
#endif

  gpio(I_HFCLOCK_RUN, mode == MODE_XTAL ? I_HFCLOCK_RUN : 0);

  dprintk("D%c", "ORCX"[mode]);

  if (pv->hf_mode == mode)
    return;

  pv->hf_mode = mode;

  switch (pv->hf_mode) {
  case MODE_XTAL:
    pv->hf_acc = pv->hfxo_acc;
    pv->src[NRF_CLOCK_HF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
    pv->src[NRF_CLOCK_HF_PRECISE].flags |= DEV_CLOCK_SRC_EP_RUNNING;
#endif
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
    switch (pv->lf_calib_state) {
    case CALIB_RUNNING:
    case CALIB_AGAIN_RUNNING:
    case CALIB_AGAIN_CHECKING:
      printk("HFCLK changed with running ?");
    case CALIB_NONE:
    case CALIB_DONE:
      break;

    case CALIB_NOTYET:
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CAL);
      pv->lf_calib_state = CALIB_RUNNING;
      dprintk("`");
      nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
      gpio(I_CAL, I_CAL);
      break;

    case CALIB_AGAIN_PENDING:
      dprintk("&");
      nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
      gpio(I_CAL, I_CAL);
      break;
    }
#endif
    break;

  case MODE_RC_PRECISE:
  case MODE_RC:
    pv->hf_acc = ACC_HF_RC;
    pv->src[NRF_CLOCK_HF].flags |= DEV_CLOCK_SRC_EP_RUNNING;
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
    pv->src[NRF_CLOCK_HF_PRECISE].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
#endif
    break;

  case MODE_OFF:
    pv->hf_acc = ACC_HF_RC;
    pv->src[NRF_CLOCK_HF].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
    pv->src[NRF_CLOCK_HF_PRECISE].flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
#endif
    break;
  }

  pv->changed_sources = (1 << NRF_CLOCK_HF);
}

static void nrf5x_clock_notify(struct nrf5x_clock_context_s *pv)
{
  static const struct dev_freq_s lf_freq = { 32768, 1 };
  static const struct dev_freq_s hf_freq = { 16000000, 1 };

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  pv->notifying = 1;

  if (pv->changed_sources & (1 << NRF_CLOCK_LF)) {
    pv->changed_sources &= ~(1 << NRF_CLOCK_LF);

    if (pv->src[NRF_CLOCK_LF_PRECISE].flags & DEV_CLOCK_SRC_EP_NOTIFY)
      dev_clock_src_changed(NULL, pv->src + NRF_CLOCK_LF_PRECISE, &lf_freq, &pv->lf_acc);

    if (pv->src[NRF_CLOCK_LF].flags & DEV_CLOCK_SRC_EP_NOTIFY)
      dev_clock_src_changed(NULL, pv->src + NRF_CLOCK_LF, &lf_freq, &pv->lf_acc);
  }

  if (pv->changed_sources & (1 << NRF_CLOCK_HF)) {
    pv->changed_sources &= ~(1 << NRF_CLOCK_HF);

    if (pv->src[NRF_CLOCK_HF_PRECISE].flags & DEV_CLOCK_SRC_EP_NOTIFY)
      dev_clock_src_changed(NULL, pv->src + NRF_CLOCK_HF_PRECISE, &hf_freq, &pv->hf_acc);

    if (pv->src[NRF_CLOCK_HF].flags & DEV_CLOCK_SRC_EP_NOTIFY)
      dev_clock_src_changed(NULL, pv->src + NRF_CLOCK_HF, &hf_freq, &pv->hf_acc);
  }

  pv->notifying = 0;

  CPU_INTERRUPT_RESTORESTATE;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_clock_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  dprintk("I");

  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED))
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED);

  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED))
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED);

  clock_lf_status_update(pv);
  clock_hf_status_update(pv);

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_CTTO)) {
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_CTTO);
    dprintk("t");

    if (pv->lf_mode != MODE_OFF) {
      switch (pv->lf_calib_state) {
      case CALIB_NONE:
      case CALIB_AGAIN_CHECKING:
      case CALIB_AGAIN_RUNNING:
      case CALIB_NOTYET:
      case CALIB_RUNNING:
        break;

      case CALIB_AGAIN_PENDING:
        hf_clock_mode_update(pv);

      case CALIB_DONE:
        pv->lf_calib_state = CALIB_AGAIN_PENDING;
        if (pv->hf_mode == MODE_XTAL) {
          nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
          gpio(I_CAL, I_CAL);
        }

      ctstart:
        nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_CTIV, 2.5 * 4);
        nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTART);
        break;
      }
    }
  }

  if (nrf_event_check(CLOCK_ADDR, NRF_CLOCK_DONE)) {
    gpio(I_CAL, 0);
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_DONE);

    dprintk(".");

    pv->lf_calib_state = CALIB_DONE;

    hf_clock_mode_update(pv);

    nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_CTIV, 2.5 * 4);
    nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTART);
  }
#endif

  nrf5x_clock_notify(pv);
}

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)

static DEV_IRQ_SRC_PROCESS(nrf5x_temp_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  dprintk("T");

  if (nrf_event_check(TEMP_ADDR, NRF_TEMP_DATARDY)) {
    nrf_event_clear(TEMP_ADDR, NRF_TEMP_DATARDY);

    gpio(I_CAL, 0);

    int32_t temp = nrf_reg_get(TEMP_ADDR, NRF_TEMP_TEMP);

    nrf_task_trigger(TEMP_ADDR, NRF_TEMP_STOP);

    if (!pv->src[NRF_CLOCK_LF_PRECISE].use_count
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_USE)
        && !pv->force[NRF_CLOCK_LF_PRECISE]
#endif
        ) {
      pv->lf_calib_state = CALIB_NONE;
      dprintk("-");
      hf_clock_mode_update(pv);
      goto notify;
    }

    if (pv->hf_mode != MODE_XTAL || pv->lf_mode == MODE_OFF) {
      dprintk("@");
      hf_clock_mode_update(pv);
      nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_CTIV, 2.5 * 4);
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTART);
      goto notify;
    }

    switch (pv->lf_calib_state) {
    case CALIB_NONE:
      dprintk("n");
      break;

    case CALIB_DONE:
    case CALIB_AGAIN_CHECKING:
    case CALIB_AGAIN_PENDING:
      if (__ABS(temp - pv->lf_calib_temp) < 2) {
        dprintk("=");
        pv->lf_calib_state = CALIB_DONE;
        hf_clock_mode_update(pv);
        nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_CTIV, 2.5 * 4);
        nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTART);
        break;
      }
      dprintk("/");
      pv->lf_calib_state = CALIB_AGAIN_RUNNING;
      goto cal;

    case CALIB_NOTYET:
      dprintk(":");
      pv->lf_calib_state = CALIB_RUNNING;
    cal:
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CAL);
      pv->lf_calib_temp = temp;
      gpio(I_CAL, I_CAL);
      break;

    case CALIB_RUNNING:
    case CALIB_AGAIN_RUNNING:
      dprintk("r");
      pv->lf_calib_temp = temp;
      break;
    }
  }

 notify:

  nrf5x_clock_notify(pv);
}

#endif

static void lf_clock_mode_update(struct nrf5x_clock_context_s *pv)
{
  enum nrf5x_clock_mode_e expected_mode = clock_lf_expected_mode_get(pv);

  dprintk("\nu");

  switch (expected_mode) {
  case MODE_OFF:
    if (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN)) {
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
      pv->lf_calib_state = CALIB_NONE ;
      pv->lf_calib_temp = -32768;
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTOP);
#endif
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
      kroutine_exec(&pv->state_checker);
    }
    break;

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  case MODE_RC_PRECISE:
    if (pv->hf_mode < MODE_XTAL) {
      pv->lf_calib_state = CALIB_NOTYET;
      hf_clock_mode_update(pv);
    } else if (pv->lf_mode == MODE_RC) {
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CAL);
      nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
      gpio(I_CAL, I_CAL);
      pv->lf_calib_state = CALIB_RUNNING;
    }
#endif

  default:
    if (!nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN)) {
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
      nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED);
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTART);
      // IRQ will enable calibration and/or notify
    }
    break;
  }
}

static void hf_clock_mode_update(struct nrf5x_clock_context_s *pv)
{
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  enum nrf5x_clock_mode_e expected_mode = clock_hf_expected_mode_get(pv);

  if (expected_mode == MODE_XTAL && !nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKRUN)) {
    nrf_event_clear(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED);
    nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTART);
    gpio(I_HFCLOCK_REQ, I_HFCLOCK_REQ);
    // IRQ will enable calibration and/or notify
  } else if (expected_mode != MODE_XTAL && nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKRUN)) {
    nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTOP);
    gpio(I_HFCLOCK_REQ, 0);
    kroutine_exec(&pv->state_checker);
  } else {
    kroutine_exec(&pv->state_checker);
  }
#endif
}

static KROUTINE_EXEC(nrf5x_clock_state_check)
{
  struct nrf5x_clock_context_s *pv = KROUTINE_CONTAINER(kr, *pv, state_checker);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  clock_lf_status_update(pv);
  clock_hf_status_update(pv);
  nrf5x_clock_notify(pv);

  CPU_INTERRUPT_RESTORESTATE;
}

static DEV_CLOCK_SRC_USE(nrf5x_clock_ep_use)
{
  struct device_s *dev = src->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;
  dev_clock_node_id_t id = src - pv->src;

  if (id > NRF_CLOCK_EP_COUNT)
    return -EINVAL;

  if (pv->notifying) {
    switch (action) {
    case DEV_CLOCK_SRC_USE_HOLD:
    case DEV_CLOCK_SRC_USE_RELEASE:
      switch (id) {
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
      case NRF_CLOCK_HF_PRECISE:
#endif
      case NRF_CLOCK_HF:
        kroutine_exec(&pv->state_checker);
        break;

      case NRF_CLOCK_LF:
      case NRF_CLOCK_LF_PRECISE:
        kroutine_exec(&pv->state_checker);
        break;
      }
    default:
      return 0;
    }

    return -EAGAIN;
  }

  switch (action) {
  case DEV_CLOCK_SRC_USE_HOLD:
    switch (id) {
    case NRF_CLOCK_HF:
      break;

#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
    case NRF_CLOCK_HF_PRECISE:
      hf_clock_mode_update(pv);

      while (synchronous && clock_hf_mode_get(pv) < MODE_XTAL)
        ;
      break;
#endif

    case NRF_CLOCK_LF_PRECISE:
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
      lf_clock_mode_update(pv);
      while (synchronous && clock_lf_mode_get(pv) < MODE_XTAL)
        ;
      break;
#endif
    case NRF_CLOCK_LF:
      lf_clock_mode_update(pv);
      while (synchronous && clock_lf_mode_get(pv) < MODE_RC)
        ;
      break;
    }
    break;

  case DEV_CLOCK_SRC_USE_RELEASE:
    switch (id) {
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
    case NRF_CLOCK_HF_PRECISE:
#endif
    case NRF_CLOCK_HF:
      hf_clock_mode_update(pv);
      break;

    case NRF_CLOCK_LF:
    case NRF_CLOCK_LF_PRECISE:
      lf_clock_mode_update(pv);
      break;
    }
    break;

  case DEV_CLOCK_SRC_USE_NOTIFY:
  case DEV_CLOCK_SRC_USE_IGNORE:
    break;
  }

  nrf5x_clock_notify(pv);

  return 0;
}

static DEV_INIT(nrf5x_clock_init);
static DEV_CLEANUP(nrf5x_clock_cleanup);
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_USE)
static DEV_USE(nrf5x_clock_use)
{
  struct device_accessor_s *accessor = param;
  struct device_s *dev = accessor->dev;
  uint_fast8_t id = accessor->number;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  printk("RTC %d dev use %d\n", accessor->number, op);

  if (op == DEV_USE_LAST_NUMBER) {
    accessor->number = NRF_CLOCK_EP_COUNT - 1;
    return 0;
  }
  
  if (id >= NRF_CLOCK_EP_COUNT)
    return -EINVAL;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    pv->force[id]++;
    break;

  case DEV_USE_STOP:
    pv->force[id]--;
    break;
  }

  switch (id) {
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  case NRF_CLOCK_HF_PRECISE:
#endif
  case NRF_CLOCK_HF:
    hf_clock_mode_update(pv);
    break;

  case NRF_CLOCK_LF:
  case NRF_CLOCK_LF_PRECISE:
    lf_clock_mode_update(pv);
    break;
  }

  return 0;
}
#else
# define nrf5x_clock_use dev_use_generic
#endif

DRIVER_DECLARE(nrf5x_clock_drv, DRIVER_FLAGS_EARLY_INIT, "nRF5x clock"
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ == 16
               " hf=16MHz"
#elif CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ == 32
               " hf=32MHz"
#else
               " hf=RC"
#endif

#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
               " lf=XO"
#else
               " lf=RC"
#endif
               , nrf5x_clock,
               DRIVER_CLOCK_METHODS(nrf5x_clock));

DRIVER_REGISTER(nrf5x_clock_drv);

static DEV_INIT(nrf5x_clock_init)
{
  struct nrf5x_clock_context_s *pv;
  uint8_t i;

  debug_init();

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_CLOCK) == addr);
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_TEMP) == addr);
#endif

  pv->lfxo_acc = ACC_LF_RC;
  pv->hfxo_acc = ACC_20PPM;
  pv->lf_mode = MODE_OFF;
  pv->hf_mode = MODE_RC;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_clock_irq);

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  device_irq_source_init(dev, pv->irq_ep + 1, 1, &nrf5x_temp_irq);
#endif

  if (device_irq_source_link(dev, pv->irq_ep,
                             _CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? 1 : 2,
                             -1))
    goto free_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);
  nrf_it_enable_mask(CLOCK_ADDR, 0
#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
                     | (1 << NRF_CLOCK_CTTO)
                     | (1 << NRF_CLOCK_DONE)
#endif
                     | (1 << NRF_CLOCK_HFCLKSTARTED)
                     | (1 << NRF_CLOCK_LFCLKSTARTED)
                     );

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)
  nrf_it_disable_mask(TEMP_ADDR, -1);
  nrf_it_enable(TEMP_ADDR, NRF_TEMP_DATARDY);
#endif

  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTOP);

  kroutine_init_sched_switch(&pv->state_checker, nrf5x_clock_state_check);

  for (i = 0; i < NRF_CLOCK_EP_COUNT; i++)
    dev_clock_source_init(dev, &pv->src[i], &nrf5x_clock_ep_use);

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(nrf5x_clock_cleanup)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);

  device_irq_source_unlink(dev, pv->irq_ep,
                           _CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? 1 : 2);

  mem_free(pv);

  return 0;
}
#endif

void arch_nrf5x_clock_init(void)
{
#if defined(CONFIG_ARCH_NRF51)
  // Always configure this, it will be useful for driver as well.
  nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_XTALFREQ,
              CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ == 16 ? NRF_CLOCK_XTALFREQ_16MHZ : NRF_CLOCK_XTALFREQ_32MHZ);
#endif

  nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_LFCLKSRC,
              _CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? NRF_CLOCK_LF_SRC_XTAL : NRF_CLOCK_LF_SRC_RC);

#if !defined(CONFIG_DRIVER_NRF5X_CLOCK)
# if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED);
# endif

  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED);
#endif
}
