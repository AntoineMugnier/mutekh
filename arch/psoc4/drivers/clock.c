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
#include <hexo/bit.h>

#include <mutek/startup.h>

#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/cmu.h>

#include <cpu/arm32m/v7m.h>

#include <arch/psoc4/srss.h>
#include <arch/psoc4/peri.h>
#include <arch/psoc4/pclk.h>
#include <arch/psoc4/sflash.h>
#include <arch/psoc4/variant.h>
#include <arch/psoc4/cpuss.h>

#define SRSS PSOC4_SRSS_ADDR
#define PERI PSOC4_PERI_ADDR
#define PCLK PSOC4_PCLK_ADDR
#define SFLASH PSOC4_SFLASH_ADDR
#define CPUSS PSOC4_CPUSS_ADDR

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#if defined(CONFIG_DRIVER_PSOC4_BLE)
enum ep_sink_e {
  SINK_ECO,
  SINK_WCO,
  SINK_COUNT,
};
#endif

static const uint32_t div_mask[] = {
  0x1fe0,
  0x1fffe0,
  0x1fffff,
  0x1fffffe0,
};

#define ILO_FREQ DEV_FREQ(32768, 1, 1, 30) // 60% accuracy

static uint_fast8_t clk_imo_trim2(uint_fast8_t freq_mhz)
{
  static const uint8_t gaps[] = {12, 24, 33, 40, 40};

  for (uint_fast8_t i = 0; i < sizeof(gaps); ++i) {
    if (freq_mhz <= gaps[i])
      return freq_mhz + i;
  }

  return freq_mhz + sizeof(gaps);
}

__attribute__((noinline))
static void psoc4_busy_wait(uint32_t loops)
{
  for (; loops; --loops)
    asm volatile("");
}

static void psoc4_flash_set_freq(uint_fast8_t freq_trim2)
{
  uint32_t select, flash_ctl;

  select = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  if (freq_trim2 > clk_imo_trim2(24))
    select |= SRSS_CLK_SELECT_HALF_EN;
  else
    select &= ~SRSS_CLK_SELECT_HALF_EN;
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, select);

  flash_ctl = cpu_mem_read_32(CPUSS + CPUSS_FLASH_CTL_ADDR);
  if (freq_trim2 <= clk_imo_trim2(16))
    CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, 0);
  else if (freq_trim2 <= clk_imo_trim2(32))
    CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, 1);
  else
    CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, 2);
  cpu_mem_write_32(CPUSS + CPUSS_FLASH_CTL_ADDR, flash_ctl);

  psoc4_busy_wait(5*4*5);
}

static
void psoc4_imo_mhz_set(uint_fast8_t freq_mhz)
{
  uint_fast32_t old_trim2 = cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR);
  uint_fast32_t imo_trim2 = SRSS_CLK_IMO_TRIM2_FREQ(clk_imo_trim2(freq_mhz));

  if (old_trim2 == imo_trim2)
    return;

  uint_fast32_t high_trim2 = clk_imo_trim2(24);
  uint_fast32_t imo_trim1 = SRSS_CLK_IMO_TRIM1_OFFSET(
    cpu_mem_read_8(SFLASH + SFLASH_IMO_TRIM_ADDR(freq_mhz - 3)));
  uint_fast32_t pwr_trim4 = SRSS_PWR_BG_TRIM4_ABS_TRIM_IMO(
    cpu_mem_read_8(SFLASH + SFLASH_IMO_ABS4_ADDR));
  uint_fast32_t pwr_trim5 = SRSS_PWR_BG_TRIM5_TMPCO_TRIM_IMO(
    cpu_mem_read_8(SFLASH + SFLASH_IMO_TMPCO4_ADDR));

  for (uint_fast8_t i = 0; i < SFLASH_IMO_MAXF_COUNT; ++i) {
    if (freq_mhz <= cpu_mem_read_8(SFLASH + SFLASH_IMO_MAXF_ADDR(i))) {
      pwr_trim4 = SRSS_PWR_BG_TRIM4_ABS_TRIM_IMO(
        cpu_mem_read_8(SFLASH + SFLASH_IMO_ABS_ADDR(i)));
      pwr_trim5 = SRSS_PWR_BG_TRIM5_TMPCO_TRIM_IMO(
        cpu_mem_read_8(SFLASH + SFLASH_IMO_TMPCO_ADDR(i)));
      
      break;
    }
  }

  dprintk("%s pwr trim4: %08x trim5: %08x, imo trim1: %08x, trim2: %08x\n",
         __FUNCTION__,
         pwr_trim4,
         pwr_trim5,
         imo_trim1,
         imo_trim2);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  psoc4_flash_set_freq(__MAX(old_trim2, imo_trim2));

  if (old_trim2 >= high_trim2 || imo_trim2 >= high_trim2) {
    dprintk("Using 24MHz temporarily\n");

    old_trim2 = SRSS_CLK_IMO_TRIM2_FREQ(clk_imo_trim2(24));
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR,
                     old_trim2);
    psoc4_busy_wait(4*5);
  }

  if (old_trim2 > imo_trim2) {
    dprintk("Setting trim2 first\n");
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR, imo_trim2);
    psoc4_busy_wait(4*5);
  }

  dprintk("Applying trims\n");
  cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM1_ADDR, imo_trim1);
  cpu_mem_write_32(SRSS + SRSS_PWR_BG_TRIM4_ADDR, pwr_trim4);
  cpu_mem_write_32(SRSS + SRSS_PWR_BG_TRIM5_ADDR, pwr_trim5);
  psoc4_busy_wait(50*4*5);

  if (old_trim2 < imo_trim2) {
    dprintk("Setting trim2\n");
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR, imo_trim2);
    psoc4_busy_wait(50*4*5);
  }

  psoc4_flash_set_freq(imo_trim2);

  CPU_INTERRUPT_RESTORESTATE;
}

#if defined(CONFIG_DRIVER_PSOC4_CLOCK)

DRIVER_PV(struct psoc4_clock_private_s
{
  struct dev_clock_src_ep_s src[PSOC4_CLOCK_SRC_COUNT];
#if defined(CONFIG_DRIVER_PSOC4_BLE)
  struct dev_clock_sink_ep_s sink[SINK_COUNT];

  struct dev_freq_s eco_freq;
  struct dev_freq_s wco_freq;
#endif
  struct dev_freq_s hfclk_freq;
  struct dev_freq_s lfclk_freq;
  struct dev_freq_s sysclk_freq;
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  struct dev_freq_s extclk_freq;
#endif

  const uint8_t *pclk_src;

  uint32_t notify_mask;
  uint32_t source_use_mask;

  /* Future register values */
#if defined(CONFIG_DRIVER_PSOC4_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  uint8_t hfclk_sel : 4;
#endif
#if defined(CONFIG_DRIVER_PSOC4_BLE)
  uint8_t lfclk_sel : 4;
#endif
  uint8_t sysclk_div;
  uint8_t imo_freq_next;
  uint8_t imo_freq;
  bool_t lfclk_running;

  struct kroutine_s updater;
  struct kroutine_s committer;
});

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

static DEV_CMU_CONFIG_MUX(psoc4_clock_config_mux)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  switch (node_id) {
  case PSOC4_CLOCK_SRC_HFCLK:
    if (ratio->num != 1 || ratio->denom != 1)
      return -ENOTSUP;

    switch (parent_id) {
#if defined(CONFIG_DRIVER_PSOC4_BLE)
    case PSOC4_CLOCK_SINK_ECO:
      pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_ECO;
      return 0;
#endif
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
    case PSOC4_CLOCK_OSC_EXTCLK:
      pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_EXTCLK;
      return 0;
#endif
    case PSOC4_CLOCK_OSC_IMO:
#if defined(CONFIG_DRIVER_PSOC4_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
      pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_IMO;
#endif
      return 0;
    default:
      return -EINVAL;
    }

  case PSOC4_CLOCK_SRC_LFCLK:
    if (ratio->num != 1 || ratio->denom != 1)
      return -ENOTSUP;

    switch (parent_id) {
#if defined(CONFIG_DRIVER_PSOC4_BLE)
    case PSOC4_CLOCK_SINK_WCO:
      pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_WCO;
      return 0;
#endif
    case PSOC4_CLOCK_OSC_ILO:
#if defined(CONFIG_DRIVER_PSOC4_BLE)
      pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_ILO;
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

#if defined(CONFIG_DRIVER_PSOC4_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR));
#endif
#if defined(CONFIG_DRIVER_PSOC4_BLE)
  pv->lfclk_sel = SRSS_WDT_CONFIG_LFCLK_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR));
#endif
}

static bool_t psoc4_pclk_is_enabled(struct device_s *dev,
                                    uint_fast8_t pclk_id)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  uint32_t div_ctl = cpu_mem_read_32(PERI + PERI_DIV_CTL_ADDR(pv->pclk_src[pclk_id]));

  return !!(div_ctl & PERI_DIV_CTL_EN);
}

static void psoc4_pclk_disable(struct device_s *dev,
                               uint_fast8_t pclk_id)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  dprintk("%s %d\n", __FUNCTION__, pclk_id);

  assert(psoc4_pclk_is_enabled(dev, pclk_id));

  cpu_mem_write_32(PERI + PERI_DIV_CMD_ADDR, 0
                   | PERI_DIV_CMD_SEL(pv->pclk_src[pclk_id])
                   | PERI_DIV_CMD_PA_SEL(PSOC4_DIV_NONE)
                   | PERI_DIV_CMD_DISABLE);

  while (cpu_mem_read_32(PERI + PERI_DIV_CMD_ADDR) & PERI_DIV_CMD_DISABLE)
    ;
}

static void psoc4_pclk_enable(struct device_s *dev,
                              uint_fast8_t pclk_id)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  dprintk("%s %d\n", __FUNCTION__, pclk_id);

  assert(!psoc4_pclk_is_enabled(dev, pclk_id));

  cpu_mem_write_32(PERI + PERI_DIV_CMD_ADDR, 0
                   | PERI_DIV_CMD_SEL(pv->pclk_src[pclk_id])
                   | PERI_DIV_CMD_PA_SEL(PSOC4_DIV_NONE)
                   | PERI_DIV_CMD_ENABLE);

  while (cpu_mem_read_32(PERI + PERI_DIV_CMD_ADDR) & PERI_DIV_CMD_ENABLE)
    ;
}

static void psoc4_pclk_div_set(struct device_s *dev,
                               uint_fast8_t pclk_id,
                               uint32_t div)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  assert(pv->pclk_src[pclk_id] != PSOC4_DIV_NONE);
  assert(div >= 0x20);

  bool_t running = psoc4_pclk_is_enabled(dev, pclk_id);
  uint32_t div_target = div - 0x20;
  uint32_t div_ctl = cpu_mem_read_32(PERI + PERI_DIV_CTL_ADDR(pv->pclk_src[pclk_id]));

  if (PERI_DIV_CTL_DIV_GET(div_ctl) == div_target)
    return;

  // Disable if not matching
  if (running)
    psoc4_pclk_disable(dev, pclk_id);

  dprintk("%s %d div: %d\n", __FUNCTION__, pclk_id, div);

  cpu_mem_write_32(PERI + PERI_DIV_CTL_ADDR(pv->pclk_src[pclk_id]),
                   PERI_DIV_CTL_DIV(div_target));

  if (running)
    psoc4_pclk_enable(dev, pclk_id);
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
#if defined(CONFIG_DRIVER_PSOC4_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))) {
  case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
    notif.freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
    running = 1;
    break;
# if defined(CONFIG_DRIVER_PSOC4_BLE)
  case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
    notif.freq = pv->eco_freq;
    running = !!(pv->sink[SINK_ECO].flags & DEV_CLOCK_EP_CLOCK);
    break;
# endif
# if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  case SRSS_CLK_SELECT_DIRECT_SEL_EXTCLK:
    notif.freq = pv->extclk_freq;
    running = 1;
    break;
# endif
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

    for (uint8_t src = 0; src <= PSOC4_CLOCK_HFBASED_LAST; ++src) {
      if (bit_get(pv->notify_mask, src))
        dev_cmu_src_notify(&pv->src[src], &notif);
    }
  }

  notif.freq = pv->lfclk_freq;
  running = 0;

#if defined(CONFIG_DRIVER_PSOC4_BLE)
  switch (SRSS_WDT_CONFIG_LFCLK_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))) {
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

  if (memcmp(&pv->lfclk_freq, &notif.freq, sizeof(notif.freq))) {
    dprintk("LFClk freq changed: %d/%d -> %d/%d\n",
            (uint32_t)pv->lfclk_freq.num,
            (uint32_t)dev_freq_acc_ppb(&pv->lfclk_freq),
            (uint32_t)notif.freq.num,
            (uint32_t)dev_freq_acc_ppb(&notif.freq));

    pv->lfclk_freq = notif.freq;

    const uint8_t src = PSOC4_CLOCK_SRC_LFCLK;

    if (bit_get(pv->notify_mask, src))
      dev_cmu_src_notify(&pv->src[src], &notif);
  }
}

static KROUTINE_EXEC(psoc4_clock_committer)
{
  struct psoc4_clock_private_s *pv = KROUTINE_CONTAINER(kr, *pv, committer);
  uint32_t tmp;
  uint_fast8_t trim2_after;

#if defined(CONFIG_DRIVER_PSOC4_BLE)
  // Enable before switch

  // Always enable ECO if selected, there is no automatic shutoff.
  if (pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_ECO) {
    dprintk("%s ECO required\n", __FUNCTION__);
    dev_clock_sink_gate(&pv->sink[SINK_ECO], DEV_CLOCK_EP_CLOCK);
  }

  // Enable WCO if we are about to select it and requested
  if (pv->lfclk_sel == SRSS_WDT_CONFIG_LFCLK_SEL_WCO
      && bit_get(pv->source_use_mask, PSOC4_CLOCK_SRC_LFCLK))
    dprintk("%s WCO required\n", __FUNCTION__);
    dev_clock_sink_gate(&pv->sink[SINK_WCO], DEV_CLOCK_EP_CLOCK);
  }
#endif

  // Enable ILO if about to use it
  if (bit_get(pv->source_use_mask, PSOC4_CLOCK_SRC_LFCLK)
#if defined(CONFIG_DRIVER_PSOC4_BLE)
      && (pv->lfclk_sel == SRSS_WDT_CONFIG_LFCLK_SEL_ILO
          || !(pv->sink[SINK_WCO].src->flags & DEV_CLOCK_EP_CLOCK))
#endif
      ) {
    dprintk("%s ILO enabled\n", __FUNCTION__);
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
    tmp |= SRSS_CLK_ILO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
  }


  // Do the switch for both clock sources
  tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  trim2_after = clk_imo_trim2(pv->imo_freq);
#if defined(CONFIG_DRIVER_PSOC4_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
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
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  case SRSS_CLK_SELECT_DIRECT_SEL_EXTCLK:
    dprintk("%s switch HFCLK to EXTCLK\n", __FUNCTION__);
    SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, EXTCLK);
    SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
    trim2_after = clk_imo_trim2(pv->extclk_freq.num / 1000000);
    break;
#endif
  }
#else
  SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, IMO);
  SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
#endif
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, tmp);
  psoc4_flash_set_freq(trim2_after);

  if (pv->source_use_mask & (1 << PSOC4_CLOCK_SRC_LFCLK)) {
    tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);
#if defined(CONFIG_DRIVER_PSOC4_BLE)
    switch (pv->lfclk_sel) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      if (!(pv->sink[SINK_WCO].flags & DEV_CLOCK_EP_CLOCK)) {
        dprintk("%s WCO not ready yet\n", __FUNCTION__);
        break;
      }
      // fallthrough
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      dprintk("%s LFCLK switched to %d\n", __FUNCTION__, pv->lfclk_sel);
      SRSS_WDT_CONFIG_LFCLK_SEL_SETVAL(tmp, pv->lfclk_sel);
      break;
    }
#else
    SRSS_WDT_CONFIG_LFCLK_SEL_SET(tmp, ILO);
#endif
    cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, tmp);
  }

#if defined(CONFIG_DRIVER_PSOC4_BLE)
  // Shutdown unused sinks
  if (pv->hfclk_sel != SRSS_CLK_SELECT_DIRECT_SEL_ECO
      && SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))
      != SRSS_CLK_SELECT_DIRECT_SEL_ECO) {
    dprintk("%s ECO unrequired\n", __FUNCTION__);
    dev_clock_sink_gate(&pv->sink[SINK_ECO], DEV_CLOCK_EP_NONE);
  }

  if ((pv->lfclk_sel != SRSS_WDT_CONFIG_LFCLK_SEL_WCO
       && (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))
           != SRSS_WDT_CONFIG_LFCLK_SEL_WCO))
      || !bit_get(pv->source_use_mask, PSOC4_CLOCK_SRC_LFCLK)) {
    dprintk("%s WCO unrequired\n", __FUNCTION__);
    dev_clock_sink_gate(&pv->sink[SINK_WCO], DEV_CLOCK_EP_NONE);
  }
#endif


  tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);
  // Disable ILO if unused
  if (!bit_get(pv->source_use_mask, PSOC4_CLOCK_SRC_LFCLK)
#if defined(CONFIG_DRIVER_PSOC4_BLE)
      || (pv->lfclk_sel != SRSS_WDT_CONFIG_LFCLK_SEL_ILO
          && (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))
              != SRSS_WDT_CONFIG_LFCLK_SEL_ILO))
#endif
      ) {
    dprintk("%s ILO disabled\n", __FUNCTION__);
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
    tmp &= ~SRSS_CLK_ILO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
  }

  // Update IMO freq
  psoc4_imo_mhz_set(pv->imo_freq_next);
  pv->imo_freq = pv->imo_freq_next;

  kroutine_exec(&pv->updater);
}

static DEV_CMU_COMMIT(psoc4_clock_commit)
{
  struct device_s *dev = accessor->dev;
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

  static const char *const node_name[] = {
    [PSOC4_CLOCK_SRC_PER_0] = "IMO SS",
    [PSOC4_CLOCK_SRC_PER_1] = "SCB0",
    [PSOC4_CLOCK_SRC_PER_2] = "SCB1",
    [PSOC4_CLOCK_SRC_PER_3] = "PUMP",
    [PSOC4_CLOCK_SRC_PER_4] = "CSD0",
    [PSOC4_CLOCK_SRC_PER_5] = "CSD1",
    [PSOC4_CLOCK_SRC_PER_6] = "SAR",
    [PSOC4_CLOCK_SRC_PER_7] = "TCPWM0",
    [PSOC4_CLOCK_SRC_PER_8] = "TCPWM1",
    [PSOC4_CLOCK_SRC_PER_9] = "TCPWM2",
    [PSOC4_CLOCK_SRC_PER_10] = "TCPWM3",
    [PSOC4_CLOCK_SRC_PER_11] = "Peri 11",
    [PSOC4_CLOCK_SRC_PER_12] = "Peri 12",
    [PSOC4_CLOCK_SRC_PER_13] = "Peri 13",
    [PSOC4_CLOCK_SRC_PER_14] = "Peri 14",
    [PSOC4_CLOCK_SRC_PER_15] = "LCD",
    [PSOC4_CLOCK_SRC_SYSCLK] = "SYSCLK",
    [PSOC4_CLOCK_SRC_HFCLK] = "HFCLK",
    [PSOC4_CLOCK_SRC_LFCLK] = "LFCLK",
#if defined(CONFIG_DRIVER_PSOC4_BLE)
    [PSOC4_CLOCK_SINK_ECO] = "ECO",
    [PSOC4_CLOCK_SINK_WCO] = "WCO",
#endif
    [PSOC4_CLOCK_OSC_IMO] = "IMO",
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
    [PSOC4_CLOCK_OSC_EXTCLK] = "ExtClk",
#endif
    [PSOC4_CLOCK_OSC_ILO] = "ILO",
  };

  if (node_id >= PSOC4_CLOCK_NODE_COUNT)
    return -EINVAL;

  if (node_id < PSOC4_CLOCK_SRC_COUNT) {
    returned |= DEV_CMU_INFO_SRC;
    info->src = &pv->src[node_id];
  }

  info->name = node_name[node_id];

  switch ((enum psoc4_clock_e)node_id) {
  case PSOC4_CLOCK_SRC_HFCLK:
    info->freq = pv->hfclk_freq;
    switch (SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))) {
    case SRSS_CLK_SELECT_DIRECT_SEL_IMO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_IMO;
      info->running = 1;
      break;
#if defined(CONFIG_DRIVER_PSOC4_BLE)
    case SRSS_CLK_SELECT_DIRECT_SEL_ECO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_SINK_ECO;
      info->running = 1;
      break;
#endif
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
    case SRSS_CLK_SELECT_DIRECT_SEL_EXTCLK:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_EXTCLK;
      info->running = 1;
      break;
#endif
    default:
      info->running = 0;
      break;
    }
    break;

  case PSOC4_CLOCK_SRC_LFCLK:
    info->freq = pv->lfclk_freq;
    switch (SRSS_WDT_CONFIG_LFCLK_SEL_GET(cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR))) {
    case SRSS_WDT_CONFIG_LFCLK_SEL_ILO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_OSC_ILO;
      info->running = !!(SRSS_CLK_ILO_CONFIG_ENABLE &
                         cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR));
      break;
#if defined(CONFIG_DRIVER_PSOC4_BLE)
    case SRSS_WDT_CONFIG_LFCLK_SEL_WCO:
      returned |= DEV_CMU_INFO_PARENT;
      info->parent_id = PSOC4_CLOCK_SINK_WCO;
      info->running = 1;
      break;
#endif
    default:
      info->running = 0;
    }
    break;

#if defined(CONFIG_DRIVER_PSOC4_BLE)
  case PSOC4_CLOCK_SINK_ECO:
    returned |= DEV_CMU_INFO_SINK;
    info->freq = pv->eco_freq;
    info->sink = &pv->sink[SINK_ECO];
    break;

  case PSOC4_CLOCK_SINK_WCO:
    returned |= DEV_CMU_INFO_SINK;
    info->freq = pv->wco_freq;
    info->sink = &pv->sink[SINK_WCO];
    break;
#endif

  case PSOC4_CLOCK_SRC_SYSCLK:
    returned |= DEV_CMU_INFO_PARENT;
    info->freq = pv->hfclk_freq;
    info->freq.num >>= SRSS_CLK_SELECT_SYSCLK_DIV_GET(
      cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR));
    info->running = 1;
    info->parent_id = PSOC4_CLOCK_SRC_HFCLK;
    break;

  case PSOC4_CLOCK_OSC_IMO:
    info->freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
    info->running = !!(cpu_mem_read_32(SRSS + SRSS_CLK_IMO_CONFIG_ADDR)
                       & SRSS_CLK_IMO_CONFIG_ENABLE);
    break;

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  case PSOC4_CLOCK_OSC_EXTCLK:
    info->freq = pv->extclk_freq;
    info->running = 1;
    break;
#endif

  case PSOC4_CLOCK_OSC_ILO:
    info->freq = ILO_FREQ;
    info->running = !!(cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR)
                       & SRSS_CLK_ILO_CONFIG_ENABLE);
    break;

  case PSOC4_CLOCK_SRC_PER_0 ... PSOC4_CLOCK_SRC_PER_15: {
    uint32_t id = node_id - PSOC4_CLOCK_SRC_PER_0;
    uint32_t div = pv->pclk_src[id];
    uint32_t div_ctl = cpu_mem_read_32(PERI + PERI_DIV_CTL_ADDR(div));

    returned |= DEV_CMU_INFO_PARENT;
    info->freq = pv->hfclk_freq;
    info->parent_id = PSOC4_CLOCK_SRC_HFCLK;
    info->running = !!(div_ctl & PERI_DIV_CTL_EN);

    if (*mask & DEV_CMU_INFO_SCALE) {
      returned |= DEV_CMU_INFO_SCALE;
      info->scale.num = 32;
      info->scale.denom = PERI_DIV_CTL_DIV_GET(div_ctl) + 0x20;
      uint64_t g = gcd64(info->scale.num, info->scale.denom);
      info->scale.num /= g;
      info->scale.denom /= g;
    }
    break;
  }

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
    pv->notify_mask |= bit(src_id);
    return 0;

  case DEV_CLOCK_SRC_SETUP_NONOTIFY:
    pv->notify_mask &= ~bit(src_id);
    return 0;
#endif

  case DEV_CLOCK_SRC_SETUP_GATES:
    if (param->flags & DEV_CLOCK_EP_CLOCK) {
      pv->source_use_mask |= 1 << src_id;
    } else {
      pv->source_use_mask &= ~(1 << src_id);

#ifdef CONFIG_DEVICE_SLEEP
      device_sleep_schedule(dev);
#endif
    }

    switch (src_id) {
    case PSOC4_CLOCK_SRC_PER_0 ... PSOC4_CLOCK_SRC_PER_15:
      dprintk("%s gates per %d; %x\n", __FUNCTION__, src_id, param->flags);

      if (!psoc4_pclk_is_enabled(dev, src_id) && (param->flags & DEV_CLOCK_EP_CLOCK))
        psoc4_pclk_enable(dev, src_id);
      else if (psoc4_pclk_is_enabled(dev, src_id) && !(param->flags & DEV_CLOCK_EP_CLOCK))
        psoc4_pclk_disable(dev, src_id);
      dev_cmu_src_update_sync(src, param->flags);
      return 0;

    case PSOC4_CLOCK_SRC_HFCLK:
      dprintk("%s gates HFCLK: %x\n", __FUNCTION__, src_id, param->flags);

      dev_cmu_src_update_sync(src, param->flags);
      return 0;

    case PSOC4_CLOCK_SRC_LFCLK:
      dprintk("%s gates LFCLK; %x\n", __FUNCTION__, param->flags);

      if (param->flags & DEV_CLOCK_EP_CLOCK) {
#if defined(CONFIG_DRIVER_PSOC4_BLE)
        kroutine_exec(&pv->committer);
        return -EAGAIN;
#endif
        uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
        tmp |= SRSS_CLK_ILO_CONFIG_ENABLE;
        cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
      } else {
#if defined(CONFIG_DRIVER_PSOC4_BLE)
        kroutine_exec(&pv->committer);
        return 0;
#endif
        uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
        tmp &= ~SRSS_CLK_ILO_CONFIG_ENABLE;
        cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
      }

      dev_cmu_src_update_sync(src, param->flags);
      return 0;
    }

  case DEV_CLOCK_SRC_SETUP_SCALER: {
    if (src_id >= PSOC4_CLOCK_PCLK_COUNT)
      return -EINVAL;

    uint8_t div_id = pv->pclk_src[src_id];
    uint32_t ratio;
    if (param->scale.num > 1)
      ratio = ((uint64_t)param->scale.denom * 32) / param->scale.num;
    else
      ratio = param->scale.denom * 32;

    dprintk("%s PCLK %d div %d.%d ratio %d\n", __FUNCTION__, src_id,
            PSOC4_DIV_TYPE(div_id), PSOC4_DIV_NO(div_id), ratio);

    if (div_id == PSOC4_DIV_NONE)
      return -ENOENT;

    if (ratio & ~div_mask[PSOC4_DIV_TYPE(div_id)])
      return -ENOTSUP;

    if (ratio < 32)
      return -EINVAL;

    psoc4_pclk_div_set(dev, src_id, ratio);

    return 0;
  }

  case DEV_CLOCK_SRC_SETUP_LINK:
    return 0;

  case DEV_CLOCK_SRC_SETUP_UNLINK:
    return 0;

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

    if ((pv->source_use_mask & PSOC4_CLOCK_SRC_NOSLEEP_MASK) == 0
#if defined(CONFIG_DRIVER_PSOC4_BLE) || defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
        && pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_IMO
        && SRSS_CLK_SELECT_DIRECT_SEL_GET(cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR))
        == SRSS_CLK_SELECT_DIRECT_SEL_IMO
#endif
        )
      cpu_mem_write_32(ARMV7M_SCR_ADDR, ARMV7M_SCR_SLEEPDEEP);

    return 0;
  }
#endif

#if defined(CONFIG_DRIVER_PSOC4_BLE)
  case DEV_USE_CLOCK_NOTIFY: {
    struct dev_clock_notify_s *notify = param;
    struct dev_clock_sink_ep_s *sink = notify->sink;
    struct device_s *dev = sink->dev;
    struct psoc4_clock_private_s *pv = dev->drv_pv;

    dprintk("PSoC4 CLOCK notify %d %d/%d\n", sink - pv->sink,
            (uint32_t)notify->freq.num, (uint32_t)notify->freq.denom);

    switch (sink - pv->sink) {
    case SINK_WCO:
      pv->wco_freq = notify->freq;
      kroutine_exec(&pv->updater);
      break;

    case SINK_ECO:
      pv->eco_freq = notify->freq;
      kroutine_exec(&pv->updater);
      break;
    }

    return 0;
  }
#endif

#if defined(CONFIG_DRIVER_PSOC4_BLE)
  case DEV_USE_CLOCK_GATES: {
    struct dev_clock_sink_ep_s *sink = param;
    struct device_s *dev = sink->dev;
    struct psoc4_clock_private_s *pv = dev->drv_pv;

    dprintk("PSoC4 CLOCK gated %d\n", sink - pv->sink);

    kroutine_exec(&pv->committer);

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
  const void *pclk_src = NULL; // Keep GCC happy
  error_t err = -1;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         SRSS == addr);
  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         PERI == addr);

  if (!pv) {
    err = device_get_param_blob(dev, "pclk_src", 0, &pclk_src);
    if (err)
      return err;

    pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
    if (!pv)
      return -ENOMEM;

    memset(pv, 0, sizeof (*pv));
    dev->drv_pv = pv;
    pv->pclk_src = pclk_src;

    // Default IMO is 24MHz, 2%
    pv->imo_freq = 24;
    psoc4_imo_mhz_set(pv->imo_freq);
    pv->hfclk_freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);
    pv->lfclk_freq = ILO_FREQ;

    kroutine_init_deferred(&pv->updater, psoc4_clock_updater);
    kroutine_init_deferred(&pv->committer, psoc4_clock_committer);
  }

  if (!(dev->init_mask & bit(0))) {
    for (uint_fast8_t i = 0; i < PSOC4_CLOCK_SRC_COUNT; ++i)
      dev_clock_source_init(dev, &pv->src[i], &psoc4_clock_ep_setup);

    for (uint_fast8_t i = 0; i < PSOC4_CLOCK_PCLK_COUNT; ++i)
      cpu_mem_write_32(PERI + PERI_PCLK_CTL_ADDR(i), pv->pclk_src[i]);

    psoc4_clock_config_read(dev);

    err = dev_cmu_init(dev, &psoc4_clock_config_ops);
    if (err)
      goto err_mem;

    device_init_enable_api(dev, 0);
  }
  
#if defined(CONFIG_DRIVER_PSOC4_BLE)
  if (bit_get(cl_missing, DRIVER_CLASS_CMU)) {
    printk("CMU deps missing\n");
    return -EAGAIN;
  }

  err = dev_drv_clock_init(dev, &pv->sink[SINK_WCO],
                           PSOC4_CLOCK_SINK_WCO, DEV_CLOCK_EP_SINK_NOTIFY,
                           &pv->wco_freq);
  if (err)
    return err;

  err = dev_drv_clock_init(dev, &pv->sink[SINK_ECO],
                           PSOC4_CLOCK_SINK_ECO, DEV_CLOCK_EP_VARFREQ | DEV_CLOCK_EP_SINK_NOTIFY,
                           &pv->eco_freq);
  if (err)
    return err;
#endif

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

DRIVER_DECLARE(psoc4_clock_drv, DRIVER_FLAGS_NO_DEPEND | DRIVER_FLAGS_EARLY_INIT | DRIVER_FLAGS_RETRY_INIT,
               "PSoC4 Clock", psoc4_clock,
               DRIVER_CMU_METHODS(psoc4_clock));

DRIVER_REGISTER(psoc4_clock_drv);

#endif

void psoc4_clock_setup(void)
{
#if !defined(CONFIG_DRIVER_PSOC4_CLOCK)
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, 0
                   | SRSS_CLK_SELECT_DIRECT_SEL(IMO)
                   );

  psoc4_imo_mhz_set(24);

  cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, 0
                   | SRSS_CLK_ILO_CONFIG_ENABLE
                   | SRSS_CLK_ILO_CONFIG_SATBIAS(SATURATED)
                   | SRSS_CLK_ILO_CONFIG_TURBO_EN
                   );
#endif
}
