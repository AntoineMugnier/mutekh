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

typedef uint16_t deps_t;

#define SRSS PSOC4_SRSS_ADDR
#define SFLASH PSOC4_SFLASH_ADDR
#define CPUSS PSOC4_CPUSS_ADDR
#define BLESS PSOC4_BLESS_ADDR
#define BLERD PSOC4_BLERD_ADDR

#define PSOC4_BLE_LL_POWER PSOC4_CLOCK_NODE_COUNT

#define DEPS_HF_IMO (bit(PSOC4_CLOCK_OSC_IMO))
#define DEPS_HF_ECO (bit(PSOC4_BLE_LL_POWER)            \
                     | bit(PSOC4_CLOCK_SRC_LFCLK)       \
                     | bit(PSOC4_CLOCK_OSC_ECO))
#define DEPS_LF_ILO (bit(PSOC4_CLOCK_OSC_ILO))
#define DEPS_LF_WCO (bit(PSOC4_CLOCK_OSC_WCO))

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
#define DEEPSLEEP_PERMITTED (bit(PSOC4_CLOCK_OSC_IMO)           \
                             | bit(PSOC4_CLOCK_OSC_WCO)         \
                             | bit(PSOC4_CLOCK_OSC_ILO)         \
                             | bit(PSOC4_CLOCK_SRC_LFCLK)       \
                             | bit(PSOC4_CLOCK_SRC_HFCLK))
#else
#define DEEPSLEEP_PERMITTED (bit(PSOC4_CLOCK_OSC_IMO)           \
                             | bit(PSOC4_CLOCK_OSC_ILO)         \
                             | bit(PSOC4_CLOCK_SRC_LFCLK)       \
                             | bit(PSOC4_CLOCK_SRC_HFCLK))
#endif

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#define ILO_FREQ DEV_FREQ(32768, 1, 1, 30) // 60% accuracy
#define IMO_FREQ(mhz) DEV_FREQ((mhz) * 1000000, 1, 2, 25)

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
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  struct dev_freq_s eco_freq;
  struct dev_freq_s wco_freq;
#endif
  deps_t node_cur_mask;
  deps_t node_notify_mask;

  /* Future register values */
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  uint8_t hfclk_sel_next : 4;
  uint8_t lfclk_sel_next : 4;
  uint8_t hfclk_sel : 4;
  uint8_t lfclk_sel : 4;
#endif
  uint8_t sysclk_div_next;
  uint8_t sysclk_div;
  uint8_t imo_trim2_next;
  uint8_t imo_trim2;

  struct kroutine_s reqs_changed;
};

DRIVER_PV(struct psoc4_clock_private_s);

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
      pv->hfclk_sel_next = SRSS_CLK_SELECT_DIRECT_SEL_IMO;
      return 0;
    case PSOC4_CLOCK_OSC_ECO:
      pv->hfclk_sel_next = SRSS_CLK_SELECT_DIRECT_SEL_ECO;
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
      pv->lfclk_sel_next = SRSS_WDT_CONFIG_LFCLK_SEL_ILO;
      return 0;
    case PSOC4_CLOCK_OSC_WCO:
      pv->lfclk_sel_next = SRSS_WDT_CONFIG_LFCLK_SEL_WCO;
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

    if (ratio->denom > 128)
      return -ENOTSUP;

    uint8_t bit = __builtin_ctz(ratio->denom);
    pv->sysclk_div_next = bit;

    return 0;
  }

  case PSOC4_CLOCK_SRC_BLELL:
    return 0;

  default:
    return -ENOENT;
  }

  return 0;
}

static DEV_CMU_ROLLBACK(psoc4_clock_rollback)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  pv->hfclk_sel_next = pv->hfclk_sel;
  pv->lfclk_sel_next = pv->lfclk_sel;
#endif
  pv->imo_trim2_next = pv->imo_trim2;
  pv->sysclk_div_next = pv->sysclk_div;

  return 0;
}

static DEV_CMU_COMMIT(psoc4_clock_commit)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  pv->hfclk_sel = pv->hfclk_sel_next;
  pv->lfclk_sel = pv->lfclk_sel_next;
#endif
  pv->imo_trim2 = pv->imo_trim2_next;
  pv->sysclk_div = pv->sysclk_div_next;

  kroutine_exec(&pv->reqs_changed);

  return 0;
}

static KROUTINE_EXEC(reqs_changed)
{
  struct psoc4_clock_private_s *pv = KROUTINE_CONTAINER(kr, *pv, reqs_changed);
  uint32_t tmp;
  deps_t node_cur_mask;
  deps_t node_need_mask = 0;
  deps_t to_start_mask;
  deps_t to_stop_mask;
  deps_t started_mask = 0;
  deps_t stopped_mask = 0;
  deps_t freq_notify = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  node_cur_mask = pv->node_cur_mask;
  CPU_INTERRUPT_RESTORESTATE;
  

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  static const uint16_t hf_deps[8] = {
    [0 ... 7] = DEPS_HF_IMO,
    [SRSS_CLK_SELECT_DIRECT_SEL_ECO] = DEPS_HF_ECO,
  };

  static const uint16_t lf_deps[4] = {
    [0 ... 3] = DEPS_LF_ILO,
    [SRSS_WDT_CONFIG_LFCLK_SEL_WCO] = DEPS_LF_WCO,
  };

  node_need_mask |= hf_deps[pv->hfclk_sel];
  node_need_mask |= hf_deps[SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))];

  if (dev_clock_src_is_clock_requested(&pv->src[PSOC4_CLOCK_SRC_BLELL]))
    node_need_mask |= DEPS_HF_ECO;

  if (dev_clock_src_is_power_requested(&pv->src[PSOC4_CLOCK_SRC_BLELL]))
    node_need_mask |= bit(PSOC4_BLE_LL_POWER) | bit(PSOC4_CLOCK_SRC_LFCLK);

  if (dev_clock_src_is_clock_requested(&pv->src[PSOC4_CLOCK_SRC_LFCLK]))
    node_need_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);

  if (node_need_mask & bit(PSOC4_CLOCK_SRC_LFCLK)) {
    node_need_mask |= lf_deps[pv->lfclk_sel];
    node_need_mask |= lf_deps[SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))];
  }
#else
  node_need_mask |= DEPS_HF_IMO | bit(PSOC4_CLOCK_SRC_HFCLK);

  if (dev_clock_src_is_clock_requested(&pv->src[PSOC4_CLOCK_SRC_LFCLK]))
    node_need_mask |= DEPS_LF_ILO | bit(PSOC4_CLOCK_SRC_LFCLK);
#endif

  to_start_mask = ~node_cur_mask & node_need_mask;
  to_stop_mask = (node_cur_mask & ~node_need_mask)
      & ~(bit(PSOC4_CLOCK_SRC_HFCLK)
          | bit(PSOC4_CLOCK_SRC_SYSCLK));

  dprintk("%s cur %04x, needs %04x, start %04x, stop %04x\n",
         __FUNCTION__,
         node_cur_mask, node_need_mask,
         to_start_mask, to_stop_mask);

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (bit_get(to_start_mask, PSOC4_CLOCK_OSC_IMO)) {
    dprintk("%s IMO enable\n", __FUNCTION__);

    uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_IMO_CONFIG_ADDR);
    tmp |= SRSS_CLK_IMO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_CONFIG_ADDR, tmp);

    started_mask |= bit(PSOC4_CLOCK_OSC_IMO);
  }
#endif

  if (bit_get(node_cur_mask, PSOC4_CLOCK_OSC_IMO)
      && pv->imo_trim2 != cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR)) {
    dprintk("%s switching IMO trim2 to %d\n", __FUNCTION__, pv->imo_trim2);
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);

    if (SRSS_CLK_SELECT_DIRECT_SEL_GET(tmp) == SRSS_CLK_SELECT_DIRECT_SEL_IMO) {
      uint_fast8_t mhz = clk_imo_mhz(pv->imo_trim2);

      psoc4_imo_mhz_set(mhz);

      freq_notify |= bit(PSOC4_CLOCK_SRC_HFCLK) | bit(PSOC4_CLOCK_SRC_SYSCLK);
    }
  }

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  if (pv->hfclk_sel != SRSS_CLK_SELECT_DIRECT_SEL_GET(tmp)) {
    uint32_t trim2_before, trim2_after, tmp;

    dprintk("%s switching HFCLK source\n", __FUNCTION__);

    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
    switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(tmp)) {
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      trim2_before = clk_imo_trim2(24);
      break;

    default:
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      trim2_before = cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR);
      break;
    }

    switch (pv->hfclk_sel) {
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      if ((node_cur_mask & DEPS_HF_ECO) != DEPS_HF_ECO)
        goto hf_not_ready;

      dprintk("%s switch HFCLK to ECO\n", __FUNCTION__);

      trim2_after = clk_imo_trim2(24);
      break;

    default:
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      if ((node_cur_mask & DEPS_HF_IMO) != DEPS_HF_IMO)
        goto hf_not_ready;

      dprintk("%s switch HFCLK to IMO\n", __FUNCTION__);
      trim2_after = cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR);
      break;
    }

    psoc4_flash_set_freq(__MAX(trim2_before, trim2_after));
    SRSS_CLK_SELECT_DIRECT_SEL_SETVAL(tmp, pv->hfclk_sel);
    cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, tmp);

    psoc4_flash_set_freq(trim2_after);

    freq_notify |=  bit(PSOC4_CLOCK_SRC_HFCLK) | bit(PSOC4_CLOCK_SRC_SYSCLK);

  hf_not_ready:
    ;
  }
#endif

  tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  if (SRSS_CLK_SELECT_SYSCLK_DIV_GET(tmp) != pv->sysclk_div) {
    dprintk("%s switching SYSCLK divisor\n", __FUNCTION__);

    SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
    cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, tmp);

    freq_notify |= bit(PSOC4_CLOCK_SRC_SYSCLK);
  }

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (bit_get(to_start_mask, PSOC4_BLE_LL_POWER)
      && bit_get(node_cur_mask, PSOC4_CLOCK_SRC_LFCLK)) {
    dprintk("%s RF Power enable\n", __FUNCTION__);

    tmp = cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR);
    tmp |= BLESS_RF_CONFIG_RF_ENABLE;
    cpu_mem_write_32(BLESS + BLESS_RF_CONFIG_ADDR, tmp);

    started_mask |= bit(PSOC4_BLE_LL_POWER);
  }

  if (bit_get(to_start_mask, PSOC4_CLOCK_OSC_ECO) && bit_get(node_cur_mask, PSOC4_BLE_LL_POWER)) {
    if (!(cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR) & BLERD_DBUS_XTAL_ENABLE)) {
      dprintk("%s ECO enable\n", __FUNCTION__);

      // Enable Put calibration values back in regs
      cpu_mem_write_32(BLERD + BLERD_BB_XO_ADDR,
                       0x2002
//                       cpu_mem_read_16(SFLASH + SFLASH_BLESS_BB_XO_ADDR)
                       );

      // Enable xtal
      tmp = cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR);
      tmp |= BLERD_DBUS_XTAL_ENABLE;
      cpu_mem_write_32(BLERD + BLERD_DBUS_ADDR, tmp);
    } else if (cpu_mem_read_32(BLERD + BLERD_FSM_ADDR) & BLERD_FSM_XO_AMP_DETECT) {
      dprintk("%s ECO enabled\n", __FUNCTION__);

      started_mask |= bit(PSOC4_CLOCK_OSC_ECO);
    } else {
      dprintk("%s ECO not ready yet\n", __FUNCTION__);
    }
  }

  if (bit_get(to_start_mask, PSOC4_CLOCK_OSC_WCO)) {
    tmp = cpu_mem_read_32(BLESS + BLESS_WCO_CONFIG_ADDR);

    if (tmp & BLESS_WCO_CONFIG_ENABLE) {
      uint32_t status = cpu_mem_read_32(BLESS + BLESS_WCO_STATUS_ADDR);
      if (status & BLESS_WCO_STATUS_OUT_BLNK_A) {
        dprintk("%s WCO ready\n", __FUNCTION__);
        started_mask |= bit(PSOC4_CLOCK_OSC_WCO);
      }
    } else {
      dprintk("%s WCO enable\n", __FUNCTION__);

      tmp |= BLESS_WCO_CONFIG_ENABLE;
      cpu_mem_write_32(BLESS + BLESS_WCO_CONFIG_ADDR, tmp);
    }
  }
#endif

  if (bit_get(to_start_mask, PSOC4_CLOCK_OSC_ILO)) {
    dprintk("%s ILO enable\n", __FUNCTION__);

    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
    tmp |= SRSS_CLK_ILO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);

    started_mask |= bit(PSOC4_CLOCK_OSC_ILO);
#if !defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    started_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);
#endif
  }

  // LFCLK switcher

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);
  if (SRSS_WDT_CONFIG_LFCLK_SEL_GET(tmp) != pv->lfclk_sel) {
    dprintk("%s updating LFCLK source\n", __FUNCTION__);

    switch (pv->lfclk_sel) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      if ((node_cur_mask & DEPS_LF_ILO) != DEPS_LF_ILO)
        goto lf_not_ready;

      dprintk("%s switch LFCLK to ILO\n", __FUNCTION__);
      break;

    default:
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      if ((node_cur_mask & DEPS_LF_WCO) != DEPS_LF_WCO)
        goto lf_not_ready;

      dprintk("%s switch LFCLK to WCO\n", __FUNCTION__);
      break;
    }

    SRSS_WDT_CONFIG_LFCLK_SEL_SETVAL(tmp, pv->lfclk_sel);
    cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, tmp);

    freq_notify |= bit(PSOC4_CLOCK_SRC_LFCLK);
  lf_not_ready:
    ;
  }

  if (bit_get(to_start_mask, PSOC4_CLOCK_SRC_LFCLK)) {
    tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);

    switch (pv->lfclk_sel) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      if (bit_get(node_cur_mask, PSOC4_CLOCK_OSC_ILO))
        started_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);
      break;

    default:
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      if (bit_get(node_cur_mask, PSOC4_CLOCK_OSC_WCO))
        started_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);
      break;
    }
  }

  if (bit_get(node_cur_mask, PSOC4_CLOCK_OSC_ECO))
    started_mask |= bit(PSOC4_CLOCK_SRC_BLELL) & ~node_cur_mask;
#endif

  // Disables

  if (bit_get(to_stop_mask, PSOC4_CLOCK_OSC_ILO)) {
    dprintk("%s ILO disable\n", __FUNCTION__);

    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
    tmp &= ~SRSS_CLK_ILO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);

    stopped_mask |= bit(PSOC4_CLOCK_OSC_ILO);
#if !defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    stopped_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);
#endif
  }

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (bit_get(to_stop_mask, PSOC4_CLOCK_OSC_WCO)) {
    dprintk("%s WCO disable\n", __FUNCTION__);

    tmp = cpu_mem_read_32(BLESS + BLESS_WCO_CONFIG_ADDR);
    tmp &= ~BLESS_WCO_CONFIG_ENABLE;
    cpu_mem_write_32(BLESS + BLESS_WCO_CONFIG_ADDR, tmp);

    stopped_mask |= bit(PSOC4_CLOCK_OSC_WCO);
  }

  if (bit_get(to_stop_mask, PSOC4_CLOCK_SRC_LFCLK)) {
    tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);

    switch (pv->lfclk_sel) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      if (bit_get(stopped_mask, PSOC4_CLOCK_OSC_ILO))
        stopped_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);
      break;

    default:
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      if (bit_get(stopped_mask, PSOC4_CLOCK_OSC_WCO))
        stopped_mask |= bit(PSOC4_CLOCK_SRC_LFCLK);
      break;
    }
  }

  if (bit_get(to_stop_mask, PSOC4_CLOCK_OSC_IMO)) {
    dprintk("%s IMO disable\n", __FUNCTION__);

    uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_IMO_CONFIG_ADDR);
    tmp &= ~SRSS_CLK_IMO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_CONFIG_ADDR, tmp);

    stopped_mask |= bit(PSOC4_CLOCK_OSC_IMO);
  }

  if (bit_get(to_stop_mask, PSOC4_CLOCK_OSC_ECO)) {
    uint32_t tmp;

    dprintk("%s ECO disable\n", __FUNCTION__);

    // Disable xtal
    tmp = cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR);
    tmp &= ~BLERD_DBUS_XTAL_ENABLE;
    cpu_mem_write_32(BLERD + BLERD_DBUS_ADDR, tmp);

    stopped_mask |= bit(PSOC4_CLOCK_OSC_ECO);
    stopped_mask |= bit(PSOC4_CLOCK_SRC_BLELL);
  }

  if (bit_get(to_stop_mask, PSOC4_BLE_LL_POWER)) {
    dprintk("%s RF Power disable\n", __FUNCTION__);

    tmp = cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR);
    tmp &= ~BLESS_RF_CONFIG_RF_ENABLE;
    cpu_mem_write_32(BLESS + BLESS_RF_CONFIG_ADDR, tmp);

    stopped_mask |= bit(PSOC4_BLE_LL_POWER);
  }
#endif

  freq_notify &= pv->node_notify_mask;

  // Freq change notifications
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (freq_notify & bit(PSOC4_CLOCK_SRC_LFCLK)) {
    struct dev_clock_notify_s lfclk;
    tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);

    switch (SRSS_WDT_CONFIG_LFCLK_SEL_GET(tmp)) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      lfclk.freq = pv->wco_freq;
      break;

    default:
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      lfclk.freq = ILO_FREQ;
      break;
    }

    if (bit_get(freq_notify, PSOC4_CLOCK_SRC_LFCLK))
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_LFCLK], &lfclk);
  }

  if (freq_notify & (bit(PSOC4_CLOCK_SRC_HFCLK) | bit(PSOC4_CLOCK_SRC_SYSCLK))) {
    struct dev_clock_notify_s hfclk;

    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
    switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(tmp)) {
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      hfclk.freq = pv->eco_freq;
      break;

    default:
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      hfclk.freq = IMO_FREQ(clk_imo_mhz(cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR)));
      break;
    }

    if (bit_get(freq_notify, PSOC4_CLOCK_SRC_HFCLK))
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_HFCLK], &hfclk);

    if (bit_get(freq_notify, PSOC4_CLOCK_SRC_SYSCLK)) {
      hfclk.freq.num >>= pv->sysclk_div;
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_SYSCLK], &hfclk);
    }
  }
#else
  if (freq_notify & (bit(PSOC4_CLOCK_SRC_HFCLK) | bit(PSOC4_CLOCK_SRC_SYSCLK))) {
    struct dev_clock_notify_s hfclk;

    hfclk.freq = IMO_FREQ(clk_imo_mhz(cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR)));

    if (bit_get(freq_notify, PSOC4_CLOCK_SRC_HFCLK))
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_HFCLK], &hfclk);

    if (bit_get(freq_notify, PSOC4_CLOCK_SRC_SYSCLK)) {
      hfclk.freq.num >>= pv->sysclk_div;
      dev_cmu_src_notify(&pv->src[PSOC4_CLOCK_SRC_SYSCLK], &hfclk);
    }
  }
#endif

  // Poll if other things are expected to change
  if (((node_need_mask ^ node_cur_mask)
       & (bit(PSOC4_CLOCK_OSC_ILO)
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
          | bit(PSOC4_CLOCK_OSC_WCO)
          | bit(PSOC4_CLOCK_OSC_ECO)
          | bit(PSOC4_BLE_LL_POWER)
#endif
          | bit(PSOC4_CLOCK_OSC_IMO)))
      || (to_stop_mask & ~stopped_mask & ~bit(PSOC4_CLOCK_SRC_BLELL))
      || (freq_notify))
    kroutine_exec(&pv->reqs_changed);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  node_cur_mask = pv->node_cur_mask;
  node_cur_mask |= started_mask;
  node_cur_mask &= ~stopped_mask;
  pv->node_cur_mask = node_cur_mask;
  CPU_INTERRUPT_RESTORESTATE;

  // Update gating changes
  deps_t changed = started_mask | stopped_mask;

  if (bit_get(changed, PSOC4_CLOCK_SRC_LFCLK))
    dev_cmu_src_update_async(&pv->src[PSOC4_CLOCK_SRC_LFCLK],
                             bit_get(node_cur_mask, PSOC4_CLOCK_SRC_LFCLK)
                             ? DEV_CLOCK_EP_CLOCK : 0);

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  if (changed & (bit(PSOC4_CLOCK_SRC_BLELL) | bit(PSOC4_BLE_LL_POWER)))
      dev_cmu_src_update_async(
        &pv->src[PSOC4_CLOCK_SRC_BLELL],
        (bit_get(node_cur_mask, PSOC4_CLOCK_SRC_BLELL) ? DEV_CLOCK_EP_CLOCK : 0)
        | (bit_get(node_cur_mask, PSOC4_BLE_LL_POWER) ? DEV_CLOCK_EP_POWER : 0));
#endif
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

    pv->imo_trim2_next = clk_imo_trim2(mhz);

    return 0;
  }

  case PSOC4_CLOCK_OSC_ILO:
    if (hz != 32768)
      return -EINVAL;

    return 0;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  case PSOC4_CLOCK_OSC_ECO:
    if (hz != 24000000)
      return -EINVAL;
    pv->eco_freq = *freq;
    return 0;

  case PSOC4_CLOCK_OSC_WCO:
    if (hz != 32768)
      return -EINVAL;
    pv->wco_freq = *freq;
    return 0;
#endif

  default:
    return -ENOENT;
  }

  return -ENOENT;
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
    [PSOC4_CLOCK_SRC_BLELL] = "BLELL",
    [PSOC4_CLOCK_OSC_ECO] = "ECO",
    [PSOC4_CLOCK_OSC_WCO] = "WCO",
#endif
    [PSOC4_CLOCK_OSC_IMO] = "IMO",
    [PSOC4_CLOCK_OSC_ILO] = "ILO",
  };

  if (node_id >= PSOC4_CLOCK_NODE_COUNT)
    return -EINVAL;

  info->name = node_name[node_id];

  if (bit_get(PSOC4_CLOCK_SRC_MASK, node_id)) {
    returned |= DEV_CMU_INFO_SRC;
    info->src = &pv->src[node_id];
  }

  switch ((enum psoc4_clock_e)node_id) {
  case PSOC4_CLOCK_SRC_SYSCLK:
    div = SRSS_CLK_SELECT_SYSCLK_DIV_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR));
  case PSOC4_CLOCK_SRC_HFCLK:
    switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))) {
    default:
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_IMO;
      info->running = 1;
      info->freq = IMO_FREQ(clk_imo_mhz(cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR)));
      break;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_ECO;
      info->running =
          (cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR) & BLESS_RF_CONFIG_RF_ENABLE)
          && (cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR) & BLERD_DBUS_XTAL_ENABLE);
      info->freq = pv->eco_freq;
      break;
#endif
    }
    info->freq.num >>= div;
    break;

  case PSOC4_CLOCK_SRC_LFCLK:
    switch (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))) {
    default:
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      returned |= DEV_CMU_INFO_PARENT;
      info->freq = ILO_FREQ;
      info->parent_id = PSOC4_CLOCK_OSC_ILO;
      info->running = !!(SRSS_CLK_ILO_CONFIG_ENABLE &
                         cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR));
      break;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_WCO;
      info->running = !!(cpu_mem_read_32(BLESS + BLESS_WCO_CONFIG_ADDR)
                         & BLESS_WCO_CONFIG_ENABLE);
      info->freq = pv->wco_freq;
      break;
#endif
    }
    break;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  case PSOC4_CLOCK_SRC_BLELL:
    returned |= DEV_CMU_INFO_SCALE;
    info->scale.num = 1;
    info->scale.denom = 1 << BLESS_XTAL_CLK_DIV_CONFIG_LLCLK_DIV_GET(
      cpu_mem_read_32(BLESS + BLESS_XTAL_CLK_DIV_CONFIG_ADDR));
  case PSOC4_CLOCK_OSC_ECO:
    info->freq = pv->eco_freq;
    info->running =
        (cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR) & BLESS_RF_CONFIG_RF_ENABLE)
        && (cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR) & BLERD_DBUS_XTAL_ENABLE);
    break;

  case PSOC4_CLOCK_OSC_WCO:
    info->freq = pv->wco_freq;
    info->running = !!(cpu_mem_read_32(BLESS + BLESS_WCO_CONFIG_ADDR)
                       & BLESS_WCO_CONFIG_ENABLE);
    break;
#endif

  case PSOC4_CLOCK_OSC_IMO:
    info->freq = IMO_FREQ(clk_imo_mhz(cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR)));
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

  case DEV_CLOCK_SRC_SETUP_GATES: {
    enum dev_clock_ep_flags_e flags = 0;

    dprintk("%s setup gates %d %02x\n", __FUNCTION__, src_id, param->flags);

    kroutine_exec(&pv->reqs_changed);

    switch (src_id) {
    case PSOC4_CLOCK_SRC_BLELL:
        flags |= bit_get(pv->node_cur_mask, PSOC4_CLOCK_OSC_ECO) ? DEV_CLOCK_EP_CLOCK : 0;
        flags |= bit_get(pv->node_cur_mask, PSOC4_BLE_LL_POWER) ? DEV_CLOCK_EP_POWER : 0;
        goto maybe;

    case PSOC4_CLOCK_SRC_LFCLK:
        flags |= bit_get(pv->node_cur_mask, PSOC4_CLOCK_SRC_LFCLK) ? DEV_CLOCK_EP_CLOCK : 0;

    maybe:
        if (~flags & param->flags & DEV_CLOCK_EP_ANY)
            return -EAGAIN;
        // fallthrough
    case PSOC4_CLOCK_SRC_HFCLK:
    case PSOC4_CLOCK_SRC_SYSCLK:
      dev_cmu_src_update_sync(src, param->flags);
      return 0;
    }

    return -EINVAL;
  }

  case DEV_CLOCK_SRC_SETUP_LINK:
    switch (src_id) {
    default:
      if (param->sink->flags & DEV_CLOCK_EP_GATING_SYNC)
        return -ENOTSUP;
      return 0;

    case PSOC4_CLOCK_SRC_HFCLK:
    case PSOC4_CLOCK_SRC_SYSCLK:
      return 0;
    }
    return 0;

  case DEV_CLOCK_SRC_SETUP_UNLINK:
    return 0;

  case DEV_CLOCK_SRC_SETUP_SCALER:
    switch (src_id) {
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
    case PSOC4_CLOCK_SRC_BLELL: {
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

    if (!(pv->node_cur_mask & ~DEEPSLEEP_PERMITTED))
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
  pv->imo_trim2 = clk_imo_trim2(24);
  psoc4_imo_mhz_set(24);
  pv->node_cur_mask = bit(PSOC4_CLOCK_OSC_IMO)
      | bit(PSOC4_CLOCK_SRC_HFCLK)
      | bit(PSOC4_CLOCK_SRC_SYSCLK);

  kroutine_init_deferred(&pv->reqs_changed, reqs_changed);

  for (uint_fast8_t i = 0; i < PSOC4_CLOCK_SRC_COUNT; ++i)
    dev_clock_source_init(dev, &pv->src[i], &psoc4_clock_ep_setup);

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_BLE)
  pv->hfclk_sel_next = pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_IMO;
  pv->lfclk_sel_next = pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_ILO;

  cpu_mem_write_32(BLESS + BLESS_WCO_TRIM_ADDR, 0
                   | BLESS_WCO_TRIM_XGM(2250_NA)
                   | BLESS_WCO_TRIM_LPM_GM(2)
                   );
#endif

  err = dev_cmu_init(dev, &psoc4_clock_config_ops);
  if (err) {
    dprintk("dev_cmu_init failed %d\n", err);
    goto err_mem;
  }

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
