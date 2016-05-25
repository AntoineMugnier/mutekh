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
# include <device/class/cmu.h>
#endif

#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/temp.h>

#define CLOCK_ADDR NRF_PERIPHERAL_ADDR(NRF5X_CLOCK)
#define TEMP_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TEMP)

#define HFXO_PRESENT (CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0)
#define LFXO_PRESENT defined(CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC)

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

#define FREQ_HF_RC DEV_FREQ(16000000, 1, 7, 20) // 983ppm
#define FREQ_HF_20PPM DEV_FREQ(16000000, 1, 2, 15)
#define FREQ_LF_RC DEV_FREQ(32768, 1, 2, 25) // 2%
#define FREQ_LF_RC_CALIBRATED DEV_FREQ(32768, 1, 0, 19) // 256ppm

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

DRIVER_PV(struct nrf5x_clock_context_s
{
  struct dev_irq_src_s irq_ep[_CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? 1 : 2];

  struct dev_clock_src_ep_s src[NRF_CLOCK_EP_COUNT];

  struct dev_freq_s hfxo_freq;
  struct dev_freq_s lfxo_freq;

  bool_t notifying;
  uint8_t running_sources;
  uint8_t requested_sources;
  uint8_t ready_sources;

  struct kroutine_s state_checker;

#if !LFXO_PRESENT
  int32_t lf_calib_temp;
  enum nrf5x_calib_state_e lf_calib_state;
#endif
  enum nrf5x_clock_mode_e lf_mode, hf_mode;
});

static void hf_clock_mode_update(struct nrf5x_clock_context_s *pv);

static DEV_CMU_NODE_INFO(nrf5x_clock_node_info)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  if (node_id >= NRF_CLOCK_EP_COUNT)
    return -EINVAL;

  *mask &= ~DEV_CMU_INFO_SINK;
  info->src = NULL;
  info->parent_id = -1;

  switch (node_id) {
# if !LFXO_PRESENT
  case NRF_CLOCK_LF:
    info->name = "LFCLK (RC)";
    info->src = &pv->src[NRF_CLOCK_LF];
    info->freq = FREQ_LF_RC;
    info->running = nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN);
    break;

  case NRF_CLOCK_LF_PRECISE:
    info->name = "LFCLK (RC, Precise)";
    info->src = &pv->src[NRF_CLOCK_LF_PRECISE];
    info->freq = FREQ_LF_RC_CALIBRATED;
    info->running = nrf_it_is_enabled(CLOCK_ADDR, NRF_CLOCK_DONE);
    break;

#else
  case NRF_CLOCK_LF:
  case NRF_CLOCK_LF_PRECISE:
    info->name = "LFCLK (XO)";
    info->src = &pv->src[NRF_CLOCK_LF_PRECISE];
    info->freq = pv->lfxo_freq;
    info->running = !!(nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT)
                       & NRF_CLOCK_LFCLKSTAT_STATE_RUNNING);
    break;
#endif

# if HFXO_PRESENT
  case NRF_CLOCK_HF_PRECISE:
    info->running = (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_HFCLKSTAT) & NRF_CLOCK_HF_SRC_MASK)
      == NRF_CLOCK_HF_SRC_XTAL;
    info->freq = pv->hfxo_freq;
    info->name = "HFCLK (precise)";
    info->src = &pv->src[NRF_CLOCK_HF_PRECISE];
    break;
#endif

  case NRF_CLOCK_HF:
    info->freq = FREQ_HF_RC;
    info->name = "HFCLK (default)";
    info->src = &pv->src[NRF_CLOCK_HF];
    info->running = 1;
    break;
  }

  return 0;
}

static enum nrf5x_clock_mode_e clock_lf_expected_mode_get(struct nrf5x_clock_context_s *pv)
{
  if (pv->requested_sources & (1 << NRF_CLOCK_LF_PRECISE)) {
#if LFXO_PRESENT
    return MODE_XTAL;
#else
    return MODE_RC_PRECISE;
#endif
  }

  if (pv->requested_sources & (1 << NRF_CLOCK_LF))
    return MODE_RC;

  return MODE_OFF;
}

static enum nrf5x_clock_mode_e clock_hf_expected_mode_get(struct nrf5x_clock_context_s *pv)
{
#if HFXO_PRESENT
  if (pv->requested_sources & (1 << NRF_CLOCK_HF_PRECISE))
    return MODE_XTAL;

# if !LFXO_PRESENT
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

static void clock_lf_status_update(struct nrf5x_clock_context_s *pv)
{
  enum nrf5x_clock_mode_e mode = MODE_OFF;

  dprintk("d");

  if (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT) & NRF_CLOCK_LFCLKSTAT_STATE_RUNNING
      && nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKRUN)) {
    switch (nrf_reg_get(CLOCK_ADDR, NRF_CLOCK_LFCLKSTAT) & NRF_CLOCK_LF_SRC_MASK) {
    case NRF_CLOCK_LF_SRC_RC:
      mode = MODE_RC;
#if !LFXO_PRESENT
      switch (pv->lf_calib_state) {
      case CALIB_AGAIN_RUNNING:
      case CALIB_AGAIN_CHECKING:
      case CALIB_DONE:
        mode = MODE_RC_PRECISE;
      case CALIB_NONE:
      case CALIB_RUNNING:
        break;

      case CALIB_AGAIN_PENDING:
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
      mode = MODE_XTAL;
      break;

    case NRF_CLOCK_LF_SRC_SYNTH:
      mode = MODE_XTAL;
      break;
    }
  }

  dprintk("%c", "orcx"[mode]);
#if !LFXO_PRESENT
  dprintk("%d", pv->lf_calib_state);
#endif

  if (pv->lf_mode == mode)
    return;

  pv->lf_mode = mode;

  pv->running_sources &= ~((1 << NRF_CLOCK_LF) | (1 << NRF_CLOCK_LF_PRECISE));

  switch (pv->lf_mode) {
  case MODE_XTAL:
  case MODE_RC_PRECISE:
    pv->running_sources |= (1 << NRF_CLOCK_LF) | (1 << NRF_CLOCK_LF_PRECISE);
    break;

  case MODE_RC:
    pv->running_sources |= 1 << NRF_CLOCK_LF;
    break;

  case MODE_OFF:
    break;
  }
}

static void clock_hf_status_update(struct nrf5x_clock_context_s *pv)
{
  enum nrf5x_clock_mode_e mode = MODE_RC;

#if HFXO_PRESENT
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

  pv->running_sources &= ~((1 << NRF_CLOCK_HF) | (1 << NRF_CLOCK_HF_PRECISE));

  switch (pv->hf_mode) {
  case MODE_XTAL:
    pv->running_sources |= (1 << NRF_CLOCK_HF) | (1 << NRF_CLOCK_HF_PRECISE);
#if !LFXO_PRESENT
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
    pv->running_sources |= 1 << NRF_CLOCK_HF;
    break;

  case MODE_OFF:
    break;
  }
}

static void nrf5x_clock_notify(struct nrf5x_clock_context_s *pv)
{
  uint8_t changed = pv->running_sources & ~pv->ready_sources;

  pv->notifying = 1;

  for (uint_fast8_t i = 0; i < NRF_CLOCK_EP_COUNT; ++i) {
    if (changed & (1 << i))
      dev_cmu_src_update_async(pv->src + i, DEV_CLOCK_EP_CLOCK);

    dev_cmu_src_update_sync(pv->src + i,
                       (pv->running_sources & (1 << i)) ? DEV_CLOCK_EP_CLOCK : 0);
  }

  pv->notifying = 0;
}

static void clock_lf_calibrate(struct nrf5x_clock_context_s *pv)
{
#if !LFXO_PRESENT
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
  clock_lf_calibrate(pv);
  nrf5x_clock_notify(pv);
}

#if !LFXO_PRESENT

static DEV_IRQ_SRC_PROCESS(nrf5x_temp_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  dprintk("T");

  if (!nrf_event_check(TEMP_ADDR, NRF_TEMP_DATARDY))
    goto notify;

  nrf_event_clear(TEMP_ADDR, NRF_TEMP_DATARDY);

  gpio(I_CAL, 0);

  int32_t temp = nrf_reg_get(TEMP_ADDR, NRF_TEMP_TEMP);

  nrf_task_trigger(TEMP_ADDR, NRF_TEMP_STOP);

  if (!(pv->requested_sources & (1 << NRF_CLOCK_LF_PRECISE))) {
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
#if !LFXO_PRESENT
      pv->lf_calib_state = CALIB_NONE ;
      pv->lf_calib_temp = -32768;
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_CTSTOP);
#endif
      nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
      kroutine_exec(&pv->state_checker);
    }
    break;

#if !LFXO_PRESENT
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
#if HFXO_PRESENT
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

static DEV_CLOCK_SRC_SETUP(nrf5x_clock_ep_setup)
{
  struct device_s *dev = src->dev;
  struct nrf5x_clock_context_s *pv = dev->drv_pv;
  dev_cmu_node_id_t id = src - pv->src;

  if (id > NRF_CLOCK_EP_COUNT)
    return -EINVAL;

  switch (op) {
  case DEV_CLOCK_SRC_SETUP_SCALER:
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  case DEV_CLOCK_SRC_SETUP_NOTIFY:
#endif
#ifdef CONFIG_DEVICE_CLOCK_THROTTLE
  case DEV_CLOCK_SRC_SETUP_THROTTLE:
    return -ENOTSUP;
#endif

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  case DEV_CLOCK_SRC_SETUP_NONOTIFY:
    return 0;
#endif

  case DEV_CLOCK_SRC_SETUP_LINK:
    if (param->sink->flags & (DEV_CLOCK_EP_GATING_SYNC | DEV_CLOCK_EP_FREQ_NOTIFY))
      return -ENOTSUP;
    return 0;

  case DEV_CLOCK_SRC_SETUP_UNLINK:
    return 0;

  case DEV_CLOCK_SRC_SETUP_GATES: {
    uint8_t mask = 1 << id;

    dev_cmu_src_update_sync(src, param->flags);

    if (param->flags & DEV_CLOCK_EP_CLOCK)
      pv->requested_sources |= mask;
    else
      pv->requested_sources &= ~mask;

    if (pv->notifying) {
      kroutine_exec(&pv->state_checker);
    } else {
      switch (id) {
      case NRF_CLOCK_HF:
        break;

#if HFXO_PRESENT
      case NRF_CLOCK_HF_PRECISE:
        hf_clock_mode_update(pv);
        break;
#endif

      case NRF_CLOCK_LF_PRECISE:
      case NRF_CLOCK_LF:
        lf_clock_mode_update(pv);
        break;
      }
    }
    break;
  }
  }

  return 0;
}


#define nrf5x_clock_config_mux (dev_cmu_config_mux_t*)dev_driver_notsup_fcn
#define nrf5x_clock_rollback (dev_cmu_rollback_t*)dev_driver_notsup_fcn
#define nrf5x_clock_app_configid_set (dev_cmu_rollback_t*)dev_driver_notsup_fcn
#define nrf5x_clock_use dev_use_generic

static DEV_CMU_COMMIT(nrf5x_clock_commit)
{
  return 0;
}

static DEV_CMU_CONFIG_OSC(nrf5x_clock_config_osc)
{
  return -ENOENT;
}

DRIVER_CMU_CONFIG_OPS_DECLARE(nrf5x_clock);

const struct driver_s nrf5x_clock_drv;

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
#if !LFXO_PRESENT
  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_TEMP) == addr);
#endif

  kroutine_init_deferred(&pv->state_checker, nrf5x_clock_state_check);

  pv->lfxo_freq = FREQ_LF_RC;
  pv->hfxo_freq = FREQ_HF_20PPM;
  pv->lf_mode = MODE_OFF;
  pv->hf_mode = MODE_RC;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_clock_irq);

#if !LFXO_PRESENT
  device_irq_source_init(dev, pv->irq_ep + 1, 1, &nrf5x_temp_irq);
#endif

  if (device_irq_source_link(dev, pv->irq_ep,
                             _CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? 1 : 2,
                             -1))
    goto free_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);
  nrf_it_enable_mask(CLOCK_ADDR, 0
#if !LFXO_PRESENT
                     | bit(NRF_CLOCK_CTTO)
                     | bit(NRF_CLOCK_DONE)
#endif
                     | bit(NRF_CLOCK_HFCLKSTARTED)
                     | bit(NRF_CLOCK_LFCLKSTARTED)
                     );

#if !LFXO_PRESENT
  nrf_it_disable_mask(TEMP_ADDR, -1);
  nrf_it_enable(TEMP_ADDR, NRF_TEMP_DATARDY);
#endif

  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTOP);
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTOP);

  for (i = 0; i < NRF_CLOCK_EP_COUNT; i++)
    dev_clock_source_init(dev, &pv->src[i], &nrf5x_clock_ep_setup);

  if (dev_cmu_init(dev, &nrf5x_clock_config_ops))
    goto free_pv;

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_clock_cleanup)
{
  struct nrf5x_clock_context_s *pv = dev->drv_pv;

  nrf_it_disable_mask(CLOCK_ADDR, -1);

  device_irq_source_unlink(dev, pv->irq_ep,
                           _CONFIG_DRIVER_NRF5X_CLOCK_LFCLK_XOSC ? 1 : 2);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_clock_drv, DRIVER_FLAGS_EARLY_INIT, "nRF5x clock"
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ == 16
               " HFXO=16MHz"
#elif CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ == 32
               " HFXO=32MHz"
#else
               " HFRC"
#endif

#if LFXO_PRESENT
               " LFXO"
#else
               " LFRC/Cal"
#endif
               , nrf5x_clock,
               DRIVER_CMU_METHODS(nrf5x_clock));

DRIVER_REGISTER(nrf5x_clock_drv);
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
# if HFXO_PRESENT
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_HFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF_CLOCK_HFCLKSTARTED);
# endif

  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED);
#endif
}

