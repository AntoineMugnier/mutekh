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
#include <hexo/bit.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/cmu.h>

#include <cpu/arm32m/v7m.h>

#include <arch/psoc4/variant.h>
#include <arch/psoc4/srss.h>
#include <arch/psoc4/sflash.h>
#include <arch/psoc4/cpuss.h>
#include <arch/psoc4/bless.h>
#include <arch/psoc4/blerd.h>

#define SRSS PSOC4_SRSS_ADDR
#define SFLASH PSOC4_SFLASH_ADDR
#define CPUSS PSOC4_CPUSS_ADDR
#define BLESS PSOC4_BLESS_ADDR
#define BLERD PSOC4_BLERD_ADDR

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#define ILO_FREQ DEV_FREQ(32768, 1, 1, 30) // 60% accuracy

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
# include "clock_ble.h"
#endif
#include "imo.h"

enum clock_state_e
{
  CLK_STATE_OFF,
  CLK_STATE_POWERING_ON,
  CLK_STATE_POWERING_OFF,
  CLK_STATE_POWERED,
  CLK_STATE_STARTING,
  CLK_STATE_STOPPING,
  CLK_STATE_RUNNING,
};

struct psoc4_clock_private_s
{
  struct dev_clock_src_ep_s src[PSOC4_CLOCK_SRC_COUNT];
  struct dev_freq_s hfclk_freq;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  struct dev_freq_s lfclk_freq;
  struct dev_freq_s eco_freq;
  struct dev_freq_s wco_freq;
#endif

  uint32_t node_notify_mask;
  uint32_t node_use_mask;
  uint32_t node_run_mask;

  /* Future register values */
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  uint8_t hfclk_sel : 4;
  uint8_t lfclk_sel : 4;
#endif
  uint8_t sysclk_div;
  uint8_t imo_freq_next;
  uint8_t imo_freq;

  struct kroutine_s lf_fsm;
  struct kroutine_s hf_fsm;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  struct kroutine_s ll_fsm;
#endif
};

DRIVER_PV(struct psoc4_clock_private_s);

static KROUTINE_EXEC(psoc4_clock_hf_fsm)
{
  struct psoc4_clock_private_s *pv = KROUTINE_CONTAINER(kr, *pv, hf_fsm);

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  uint32_t sel, tmp;
  uint32_t trim2_after;

  if (pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_IMO) {
    trim2_after = clk_imo_trim2(pv->imo_freq);

  }

  tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  if (pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_GET(tmp)) {
    return;
  } else if (pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_ECO) {
    kroutine_exec(&pv->ll_fsm);
    return;
  }

  if (bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_LFCLK)
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
      || bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_BLE_LL)
#endif
      ) {
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    switch (pv->lfclk_sel) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO: {
      dprintk("%s WCO required\n", __FUNCTION__);
      can_switch_lfclk &= psoc4_clock_wco_start();
      break;
    }

    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      dprintk("%s ILO enabled\n", __FUNCTION__);
      psoc4_clock_ilo_start();
      break;
    }
#else
    dprintk("%s ILO enabled\n", __FUNCTION__);
    psoc4_clock_ilo_start();
#endif
  }

  // Do the switch
  if (can_switch_hfclk) {
    uint32_t trim2_after, tmp;
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
    trim2_after = clk_imo_trim2(pv->imo_freq);
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    switch (pv->hfclk_sel) {
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      if (!(pv->sink[SINK_ECO].src->flags & DEV_CLOCK_EP_CLOCK)) {
        dprintk("%s ECO not ready yet\n", __FUNCTION__);
        break;
      }
      dprintk("%s switch HFCLK to ECO\n", __FUNCTION__);
      psoc4_flash_set_freq(clk_imo_trim2(__MAX(24, pv->imo_freq)));
      SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, ECO);
      SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
      trim2_after = clk_imo_trim2(24);
      break;
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      dprintk("%s switch HFCLK to IMO\n", __FUNCTION__);
      psoc4_flash_set_freq(clk_imo_trim2(__MAX(pv->hfclk_freq.num / 1000000, pv->imo_freq)));
      SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, IMO);
      SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
      trim2_after = clk_imo_trim2(pv->imo_freq);
      break;
    }
#else
    SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, IMO);
    SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
#endif
    cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, tmp);
    psoc4_flash_set_freq(trim2_after);
  }

}

static DEV_CMU_CONFIG_MUX(psoc4_clock_config_mux)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  switch (node_id) {
  case PSOC4_CLOCK_SRC_HFCLK:
    if (ratio->num != 1 || ratio->denom != 1)
      return -ENOTSUP;

    switch (parent_id) {
    case PSOC4_CLOCK_OSC_IMO:
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
      pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_IMO;
      return 0;
    case PSOC4_CLOCK_SINK_ECO:
      pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_ECO;
#endif
      return 0;
    default:
      return -EINVAL;
    }

  case PSOC4_CLOCK_SRC_LFCLK:
    if (ratio->num != 1 || ratio->denom != 1)
      return -ENOTSUP;

    switch (parent_id) {
    case PSOC4_CLOCK_OSC_ILO:
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
      pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_ILO;
      return 0;
    case PSOC4_CLOCK_SINK_WCO:
      pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_WCO;
#endif
      return 0;
    default:
      return -EINVAL;
    }

  case PSOC4_CLOCK_SRC_SYSCLK: {
    if (ratio->num != 1)
      return -ENOTSUP;

    if (!is_pow2(ratio->denom))
      return -ENOTSUP;

    if (!ratio->denom > 128)
      return -ENOTSUP;

    uint8_t bit = __builtin_ctz(ratio->denom);
    pv->sysclk_div = bit;

    return 0;
  }

  default:
    return -ENOENT;
  }

  return 0;
}

static void psoc4_clock_config_read(struct device_s *dev)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  (void)pv;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR));

  pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR));
#endif
}

static DEV_CMU_ROLLBACK(psoc4_clock_rollback)
{
  psoc4_clock_config_read(dev);

  return 0;
}

static KROUTINE_EXEC(psoc4_clock_updater)
{
  struct psoc4_clock_private_s *pv = KROUTINE_CONTAINER(kr, *pv, updater);
  struct dev_clock_notify_s notif;
  bool_t running = 0;

  notif.freq = pv->hfclk_freq;

  // Update internal caches
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))) {
  case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
    notif.freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
    running = 1;
    break;
  case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
    notif.freq = pv->eco_freq;
    running = psoc4_clock_eco_is_running();
    break;
  }
#else
  notif.freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
#endif

  if (memcmp(&pv->hfclk_freq, &notif.freq, sizeof(notif.freq))) {
    dprintk("HFClk freq changed: %d/%d -> %d/%d\n",
            (uint32_t)pv->hfclk_freq.num,
            (uint32_t)dev_freq_acc_ppb(&pv->hfclk_freq),
            (uint32_t)notif.freq.num,
            (uint32_t)dev_freq_acc_ppb(&notif.freq));

    pv->hfclk_freq = notif.freq;

    if (bit_get(pv->node_notify_mask, PSOC4_CLOCK_SRC_HFCLK))
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_HFCLK], &notif);
    if (bit_get(pv->node_notify_mask, PSOC4_CLOCK_SRC_SYSCLK))
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_SYSCLK], &notif);
  }

  running = 0;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  notif.freq = pv->lfclk_freq;
  switch (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))) {
  case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
    notif.freq = pv->wco_freq;
    running = !!(pv->sink[SINK_ECO].flags & DEV_CLOCK_EP_CLOCK);
    break;
  case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
    notif.freq = ILO_FREQ;
    running = !!(cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR)
                 & SRSS_CLK_ILO_CONFIG_ENABLE);
    break;
  }
#endif

  if (running != pv->lfclk_running) {
    const uint8_t src = PSOC4_CLOCK_SRC_LFCLK;

    dprintk("LFCLK now %s\n", running ? "running" : "idle");

    pv->lfclk_running = running;

    dev_cmu_src_update_async(&pv->src[src], running ? DEV_CLOCK_EP_CLOCK : 0);
  }

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (memcmp(&pv->lfclk_freq, &notif.freq, sizeof(notif.freq))) {
    dprintk("LFClk freq changed: %d/%d -> %d/%d\n",
            (uint32_t)pv->lfclk_freq.num,
            (uint32_t)dev_freq_acc_ppb(&pv->lfclk_freq),
            (uint32_t)notif.freq.num,
            (uint32_t)dev_freq_acc_ppb(&notif.freq));

    pv->lfclk_freq = notif.freq;

    const uint8_t src = PSOC4_CLOCK_SRC_LFCLK;

    if (bit_get(pv->node_notify_mask, src))
      dev_cmu_src_notify(&pv->src[src], &notif);
  }
#endif
}

static DEV_IRQ_SRC_PROCESS(psoc4_bless_irq)
{
  struct device_s *dev = ep->base.dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t tmp;

  tmp = cpu_mem_read_32(BLESS + BLESS_LL_DSM_INTR_STAT_ADDR);

  if (tmp & BLESS_LL_DSM_INTR_STAT_XTAL_ON_INTR) {
    psoc4_ble_eco_irq_handle(pv, 0);
  }
}

static
void psoc4_clock_ilo_start(void)
{
  uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
  tmp |= SRSS_CLK_ILO_CONFIG_ENABLE;
  cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
}

static
void psoc4_clock_ilo_stop(void)
{

}

static bool_t psoc4_clock_lfclk_is_running(void)
{

}

static void psoc4_clock_lfclk_start(void)
{

}

static KROUTINE_EXEC(psoc4_clock_committer)
{
  struct psoc4_clock_private_s *pv = KROUTINE_CONTAINER(kr, *pv, committer);
  struct device_s *dev = pv->src->dev;
  bool_t can_switch_lfclk = 1;
  bool_t can_switch_hfclk = 1;

  // Enable what is needed
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_ECO
      || bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_BLE_LL)) {
    dprintk("%s ECO required\n", __FUNCTION__);

    bool_t could_activate = psoc4_clock_eco_start();

    if (pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_ECO && !could_activate)
      can_switch_hfclk = 0;
  }
#endif

  if (bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_LFCLK)
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
      || bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_BLE_LL)
#endif
      ) {
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    switch (pv->lfclk_sel) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO: {
      dprintk("%s WCO required\n", __FUNCTION__);
      can_switch_lfclk &= psoc4_clock_wco_start();
      break;
    }

    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      dprintk("%s ILO enabled\n", __FUNCTION__);
      psoc4_clock_ilo_start();
      break;
    }
#else
    dprintk("%s ILO enabled\n", __FUNCTION__);
    psoc4_clock_ilo_start();
#endif
  }

  // Do the switch
  if (can_switch_hfclk) {
    uint32_t trim2_after, tmp;
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
    trim2_after = clk_imo_trim2(pv->imo_freq);
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    switch (pv->hfclk_sel) {
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      if (!(pv->sink[SINK_ECO].src->flags & DEV_CLOCK_EP_CLOCK)) {
        dprintk("%s ECO not ready yet\n", __FUNCTION__);
        break;
      }
      dprintk("%s switch HFCLK to ECO\n", __FUNCTION__);
      psoc4_flash_set_freq(clk_imo_trim2(__MAX(24, pv->imo_freq)));
      SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, ECO);
      SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
      trim2_after = clk_imo_trim2(24);
      break;
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      dprintk("%s switch HFCLK to IMO\n", __FUNCTION__);
      psoc4_flash_set_freq(clk_imo_trim2(__MAX(pv->hfclk_freq.num / 1000000, pv->imo_freq)));
      SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, IMO);
      SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
      trim2_after = clk_imo_trim2(pv->imo_freq);
      break;
    }
#else
    SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, IMO);
    SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
#endif
    cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, tmp);
    psoc4_flash_set_freq(trim2_after);
  }

  if ((bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_LFCLK)
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
       || bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_BLE_LL)
#endif
       ) && can_switch_lfclk) {
    uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    SRSS_WDT_CONFIG_LFCLK_SEL_SET(tmp, pv->lfclk_sel);
#else
    SRSS_WDT_CONFIG_LFCLK_SEL_SET(tmp, ILO);
#endif
    cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, tmp);
  }

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  // Shutdown unused sinks
  if (pv->hfclk_sel != SRSS_CLK_SELECT_DIRECT_SEL_ECO
      && !bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_BLE_LL)
      && (SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))
          != SRSS_CLK_SELECT_DIRECT_SEL_ECO)) {
    dprintk("%s ECO unrequired\n", __FUNCTION__);
    psoc4_clock_eco_stop();
  }

  if (!bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_LFCLK)
      && !bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_BLE_LL)
      && !psoc4_blerd_is_powered()) {
    if (pv->lfclk_sel != SRSS_WDT_CONFIG_LFCLK_SEL_WCO
          && (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))
              != SRSS_WDT_CONFIG_LFCLK_SEL_WCO)) {
      dprintk("%s WCO unrequired\n", __FUNCTION__);
      psoc4_clock_wco_stop();
    }

    if ((pv->lfclk_sel != SRSS_WDT_CONFIG_LFCLK_SEL_ILO
         && (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))
             != SRSS_WDT_CONFIG_LFCLK_SEL_ILO))) {
      dprintk("%s ILO unrequired\n", __FUNCTION__);
      psoc4_clock_ilo_stop();
    }
  }
#else
  if (!bit_get(pv->node_use_mask, PSOC4_CLOCK_SRC_LFCLK)) {
    dprintk("%s ILO unrequired\n", __FUNCTION__);
    psoc4_clock_ilo_stop();
  }
#endif

  // Update IMO freq
  psoc4_imo_mhz_set(pv->imo_freq_next);
  pv->imo_freq = pv->imo_freq_next;

  kroutine_exec(&pv->updater);
}

static DEV_CMU_CONFIG_OSC(psoc4_clock_config_osc)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;
  uint32_t hz = freq->num;

  if (freq->denom != 1)
    hz /= freq->denom;

  switch (node_id) {
  case PSOC4_CLOCK_OSC_IMO: {
    uint32_t mhz = hz / 1000000;

    if (mhz * 1000000 != hz)
      return -EINVAL;

    if (mhz < 3 || mhz > 48)
      return -EINVAL;

    pv->imo_freq_next = mhz;

    return 0;
  }

  case PSOC4_CLOCK_OSC_ILO:
    if (hz != 32768)
      return -EINVAL;

    return 0;

  default:
    return -ENOENT;
  }

  return -ENOENT;
}

static DEV_CMU_COMMIT(psoc4_clock_commit)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  kroutine_exec(&pv->committer);

  return 0;
}

static DEV_CMU_NODE_INFO(psoc4_clock_node_info)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_clock_private_s *pv = dev->drv_pv;
  uint32_t returned = 0
    | DEV_CMU_INFO_FREQ
    | DEV_CMU_INFO_NAME
    | DEV_CMU_INFO_RUNNING
    | DEV_CMU_INFO_ACCURACY
    ;
  uint32_t div = 1;

  static const char *const node_name[] = {
    [PSOC4_CLOCK_SRC_SYSCLK] = "SYSCLK",
    [PSOC4_CLOCK_SRC_HFCLK] = "HFCLK",
    [PSOC4_CLOCK_SRC_LFCLK] = "LFCLK",
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    [PSOC4_CLOCK_SRC_BLE_LL] = "BLE LL",
    [PSOC4_CLOCK_SINK_ECO] = "ECO",
    [PSOC4_CLOCK_SINK_WCO] = "WCO",
#endif
    [PSOC4_CLOCK_OSC_IMO] = "IMO",
    [PSOC4_CLOCK_OSC_ILO] = "ILO",
  };

  if (node_id >= PSOC4_CLOCK_NODE_COUNT)
    return -EINVAL;

  info->name = node_name[node_id];

  if (node_id & PSOC4_CLOCK_SRC_MASK) {
    returned |= DEV_CMU_INFO_SRC;
    info->src = &pv->src[node_id];
  }

  switch ((enum psoc4_clock_e)node_id) {
  case PSOC4_CLOCK_SRC_SYSCLK:
    div = SRSS_CLK_SELECT_SYSCLK_DIV_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR));
  case PSOC4_CLOCK_SRC_HFCLK:
    info->freq = pv->hfclk_freq;
    info->freq.num >>= div;
    switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))) {
    default:
      assert(0);
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_IMO;
      info->running = 1;
      break;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_SINK_ECO;
      info->running = 1;
      break;
#endif
    }
    break;

  case PSOC4_CLOCK_SRC_LFCLK:
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    info->freq = pv->lfclk_freq;
#else
    info->freq = ILO_FREQ;
#endif
    switch (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))) {
    default:
      assert(0);
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_ILO;
      info->running = !!(SRSS_CLK_ILO_CONFIG_ENABLE &
                         cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR));
      break;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_SINK_WCO;
      info->running = 1;
      break;
#endif
    }
    break;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  case PSOC4_CLOCK_SRC_BLE_LL:
    returned |= DEV_CMU_INFO_SCALE;
    info->scaler.num = 1;
    info->scaler.denom = 1 << BLESS_XTAL_CLK_DIV_CONFIG_LLCLK_DIV_GET(
      cpu_mem_read_32(BLESS + BLESS_XTAL_CLK_DIV_CONFIG_ADDR));
  case PSOC4_CLOCK_SRC_ECO:
    info->freq = pv->eco_freq;
    info->running = psoc4_clock_eco_is_running();
    break;

  case PSOC4_CLOCK_SRC_WCO:
    info->freq = pv->wco_freq;
    info->running = psoc4_clock_wco_is_running();
    break;
#endif

  case PSOC4_CLOCK_OSC_IMO:
    info->freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
    info->running = !!(cpu_mem_read_32(SRSS + SRSS_CLK_IMO_CONFIG_ADDR)
                       & SRSS_CLK_IMO_CONFIG_ENABLE);
    break;

  case PSOC4_CLOCK_OSC_ILO:
    info->freq = ILO_FREQ;
    info->running = !!(cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR)
                       & SRSS_CLK_ILO_CONFIG_ENABLE);
    break;

  default:
    return -ENOENT;
  }

  *mask &= returned;

  if (info->freq.denom > 1 && *mask & DEV_CMU_INFO_FREQ) {
    uint64_t g = gcd64(info->freq.num, info->freq.denom);
    info->freq.num /= g;
    info->freq.denom /= g;
  }

  return 0;
}

static DEV_CLOCK_SRC_SETUP(psoc4_clock_ep_setup)
{
  struct device_s *dev = src->dev;
  struct psoc4_clock_private_s *pv = dev->drv_pv;
  uint_fast8_t src_id = src - pv->src;

  if (src_id >= PSOC4_CLOCK_SRC_COUNT)
    return -EINVAL;

  switch (op) {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  case DEV_CLOCK_SRC_SETUP_NOTIFY:
    pv->node_notify_mask |= bit(src_id);
    return 0;

  case DEV_CLOCK_SRC_SETUP_NONOTIFY:
    pv->node_notify_mask &= ~bit(src_id);
    return 0;
#endif

  case DEV_CLOCK_SRC_SETUP_GATES:
    if (param->flags & DEV_CLOCK_EP_CLOCK) {
      pv->node_use_mask |= 1 << src_id;
    } else {
      pv->node_use_mask &= ~(1 << src_id);
#ifdef CONFIG_DEVICE_SLEEP
      device_sleep_schedule(dev);
#endif
    }

    switch (src_id) {
    case PSOC4_CLOCK_SRC_HFCLK:
    case PSOC4_CLOCK_SRC_SYSCLK:
      dprintk("%s gates HFCLK: %x\n", __FUNCTION__, src_id, param->flags);

      dev_cmu_src_update_sync(src, param->flags);
      return 0;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case PSOC4_CLOCK_SRC_BLE_LL:
      if (psoc4_clock_eco_start())
        dev_cmu_src_update_sync(src, param->flags);
      break;
#endif

    case PSOC4_CLOCK_SRC_LFCLK: {
      dprintk("%s gates LFCLK; %x\n", __FUNCTION__, param->flags);

      if (psoc4_clock_lfclk_is_running()) {
        dev_cmu_src_update_sync(src, param->flags);
        return 0;
      }

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
      uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);
      SRSS_WDT_CONFIG_LFCLK_SEL_SET(tmp, pv->lfclk_sel);
      cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, tmp);

      switch (pv->lfclk_sel) {
      case SRSS_WDT_CONFIG_LFCLK_SEL_WCO: {
        dprintk("%s WCO required\n", __FUNCTION__);
        psoc4_clock_wco_start();
        break;
      }

      case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
#endif
        dprintk("%s ILO required\n", __FUNCTION__);
        psoc4_clock_ilo_start();
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
        break;
      }
#endif

      kroutine_exec(&pv->committer);
      return 0;
    }
    }

  case DEV_CLOCK_SRC_SETUP_LINK:
    if (param->sink->flags & DEV_CLOCK_EP_GATING_SYNC)
      return -ENOTSUP;
    return 0;

  case DEV_CLOCK_SRC_SETUP_UNLINK:
    return 0;

  case DEV_CLOCK_SRC_SETUP_SCALER:
    switch (src_id) {
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case PSOC4_CLOCK_SRC_BLE_LL: {
      uint32_t div = param->scale.denom;

      if (param->scale.num > 1)
        div /= param->scale.num;

      if (!is_pow2(div))
        return -EINVAL;

      if (div > 8)
        return -EINVAL;

      uint32_t tmp = cpu_mem_read_32(BLESS + BLESS_XTAL_CLK_DIV_CONFIG_ADDR);
      BLESS_XTAL_CLK_DIV_CONFIG_LLCLK_DIV_SETVAL(tmp, __builtin_ctz(div));
      cpu_mem_write_32(BLESS + BLESS_XTAL_CLK_DIV_CONFIG_ADDR, tmp);
      return 0;
    }
#endif
    default:
      return -ENOTSUP;
    }
    break;

  default:
    return -ENOTSUP;
  }
}

static DEV_USE(psoc4_clock_use)
{
  switch (op) {
#ifdef CONFIG_DEVICE_SLEEP
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct psoc4_clock_private_s *pv = dev->drv_pv;

    if ((pv->node_use_mask & PSOC4_CLOCK_HFBASED_MASK) == 0
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
        && pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_IMO
        && !psoc4_clock_eco_is_running()
#endif
        )
      cpu_mem_write_32(ARMV7M_SCR_ADDR, ARMV7M_SCR_SLEEPDEEP);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

const struct driver_s psoc4_clock_drv;

DRIVER_CMU_CONFIG_OPS_DECLARE(psoc4_clock);

static DEV_INIT(psoc4_clock_init)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;
  error_t err = -1;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         SRSS == addr);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof (*pv));
  dev->drv_pv = pv;

  // Default IMO is 24MHz, 2%
  pv->imo_freq = 24;
  psoc4_imo_mhz_set(pv->imo_freq);
  pv->hfclk_freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  pv->lfclk_freq = ILO_FREQ;
#endif

  kroutine_init_deferred(&pv->updater, psoc4_clock_updater);
  kroutine_init_deferred(&pv->committer, psoc4_clock_committer);

  for (uint_fast8_t i = 0; i < PSOC4_CLOCK_SRC_COUNT; ++i)
    dev_clock_source_init(dev, &pv->src[i], &psoc4_clock_ep_setup);

  psoc4_clock_config_read(dev);

  err = dev_cmu_init(dev, &psoc4_clock_config_ops);
  if (err)
    goto err_mem;

  return 0;

 err_mem:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(psoc4_clock_cleanup)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

static DEV_CMU_APP_CONFIGID_SET(psoc4_clock_app_configid_set)
{
  struct device_s *dev = accessor->dev;
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);
  err = dev_cmu_configid_set(dev, &psoc4_clock_config_ops, config_id);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

DRIVER_DECLARE(psoc4_clock_drv, DRIVER_FLAGS_EARLY_INIT,
               "PSoC4 Clock", psoc4_clock,
               DRIVER_CMU_METHODS(psoc4_clock));

DRIVER_REGISTER(psoc4_clock_drv);
