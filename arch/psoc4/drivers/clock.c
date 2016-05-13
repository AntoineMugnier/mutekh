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
  uint32_t select, flash_ctl;

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

  dprintk("Setting flash half\n");
  select = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  select |= SRSS_CLK_SELECT_HALF_EN;
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, select);

  flash_ctl = cpu_mem_read_32(CPUSS + CPUSS_FLASH_CTL_ADDR);
  CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, 2);
  cpu_mem_write_32(CPUSS + CPUSS_FLASH_CTL_ADDR, flash_ctl);

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

  CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, freq_mhz / 16);
  cpu_mem_write_32(CPUSS + CPUSS_FLASH_CTL_ADDR, flash_ctl);

  if (imo_trim2 < high_trim2) {
    dprintk("Removing flash half\n");
    select &= ~SRSS_CLK_SELECT_HALF_EN;
    cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, select);
    psoc4_busy_wait(5*4*5);
  }

  CPU_INTERRUPT_RESTORESTATE;
}

#if defined(CONFIG_DRIVER_PSOC4_CLOCK)

struct psoc4_clock_private_s
{
  struct dev_clock_src_ep_s src[PSOC4_CLOCK_SRC_COUNT];
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
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  uint8_t hfclk_sel : 4;
#endif
  uint8_t sysclk_div;
  uint8_t imo_freq_next;
  uint8_t imo_freq;
};

static DEV_CMU_CONFIG_OSC(psoc4_clock_config_osc)
{
  struct device_s *dev = accessor->dev;
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
  struct device_s *dev = accessor->dev;
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  switch (node_id) {
  case PSOC4_CLOCK_SRC_HFCLK:
    if (ratio->num != 1 || ratio->denom != 1)
      return -ENOTSUP;

    switch (parent_id) {
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
    case PSOC4_CLOCK_OSC_EXTCLK:
      pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_EXTCLK;
      return 0;
#endif
    case PSOC4_CLOCK_OSC_IMO:
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
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
    case PSOC4_CLOCK_OSC_ILO:
      return 0;
    default:
      return -EINVAL;
    }

  case PSOC4_CLOCK_SRC_SYSCLK: {
    if (ratio->num != 1)
      return -ENOTSUP;

    if (!ALIGN_ISPOWTWO(ratio->denom))
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

#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  pv->hfclk_sel = SRSS_CLK_SELECT_DIRECT_SEL_GET(
    cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR));
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

  // Disable if not matching
  if (running && (PERI_DIV_CTL_DIV_GET(div_ctl) != div_target))
    psoc4_pclk_disable(dev, pclk_id);

  dprintk("%s %d div: %d\n", __FUNCTION__, pclk_id, div);

  cpu_mem_write_32(PERI + PERI_DIV_CTL_ADDR(pv->pclk_src[pclk_id]),
                   PERI_DIV_CTL_DIV(div_target));

  if (running)
    psoc4_pclk_enable(dev, pclk_id);
}

static DEV_CMU_ROLLBACK(psoc4_clock_rollback)
{
  struct device_s *dev = accessor->dev;

  psoc4_clock_config_read(dev);

  return 0;
}

static DEV_CMU_COMMIT(psoc4_clock_commit)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_clock_private_s *pv = dev->drv_pv;
  uint32_t tmp;
  struct dev_clock_notify_s hfclk = {
    .freq = pv->hfclk_freq,
  };

  // Enable ILO if about to use it
  if ((pv->source_use_mask & (1 << PSOC4_CLOCK_SRC_LFCLK))) {
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
    tmp |= SRSS_CLK_ILO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
  }

  // Do the switch for both clock sources
  tmp = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  SRSS_CLK_SELECT_DIRECT_SEL_SETVAL(tmp, pv->hfclk_sel);
#else
  SRSS_CLK_SELECT_DIRECT_SEL_SET(tmp, IMO);
#endif
  SRSS_CLK_SELECT_SYSCLK_DIV_SET(tmp, pv->sysclk_div);
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, tmp);

  tmp = cpu_mem_read_32(SRSS + SRSS_WDT_CONFIG_ADDR);
  SRSS_WDT_CONFIG_LFCLK_SEL_SET(tmp, ILO);
  cpu_mem_write_32(SRSS + SRSS_WDT_CONFIG_ADDR, tmp);

  // Disable ILO if unused
  if (!(pv->source_use_mask & (1 << PSOC4_CLOCK_SRC_LFCLK))) {
    tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
    tmp &= ~SRSS_CLK_ILO_CONFIG_ENABLE;
    cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
  }

  // Update IMO freq
  psoc4_imo_mhz_set(pv->imo_freq_next);
  pv->imo_freq = pv->imo_freq_next;

  // Update internal caches
  hfclk.freq = DEV_FREQ(pv->imo_freq * 1000000, 1, 2, 25);

  dprintk("HFClk freq changed: %d -> %d\n", pv->hfclk_freq.num, hfclk.freq.num);

  if (memcmp(&pv->hfclk_freq, &hfclk.freq, sizeof(hfclk.freq))) {

    pv->hfclk_freq = hfclk.freq;

    for (uint8_t src = 0; src < PSOC4_CLOCK_SRC_COUNT; ++src) {
      if (pv->notify_mask & (1 << src)) {
        dev_cmu_src_notify(&pv->src[src], &hfclk);
      }
    }
  }

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
    default:
      info->running = 0;
    }
    break;

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

  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_CLOCK_SETUP_NOTIFY:
      pv->notify_mask |= 1 << src_id;
      return 0;

    case DEV_CLOCK_SETUP_NONOTIFY:
      pv->notify_mask &= ~(1 << src_id);
      return 0;
#endif

    case DEV_CLOCK_SETUP_GATES:
      dprintk("%s gates src %d; %x\n",
             __FUNCTION__, src_id, param->flags);

      switch (src_id) {
      case PSOC4_CLOCK_SRC_PER_0 ... PSOC4_CLOCK_SRC_PER_15:
        if (!psoc4_pclk_is_enabled(dev, src_id) && (param->flags & DEV_CLOCK_EP_CLOCK))
          psoc4_pclk_enable(dev, src_id);
        else if (psoc4_pclk_is_enabled(dev, src_id) && !(param->flags & DEV_CLOCK_EP_CLOCK))
          psoc4_pclk_disable(dev, src_id);
        break;

      case PSOC4_CLOCK_SRC_LFCLK:
        if (param->flags & DEV_CLOCK_EP_CLOCK) {
          uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
          tmp |= SRSS_CLK_ILO_CONFIG_ENABLE;
          cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
        } else {
          uint32_t tmp = cpu_mem_read_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR);
          tmp &= ~SRSS_CLK_ILO_CONFIG_ENABLE;
          cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, tmp);
        }
        break;
      }

      if (param->flags & DEV_CLOCK_EP_CLOCK) {
        assert(!(pv->source_use_mask & (1 << src_id)));
        pv->source_use_mask |= 1 << src_id;
      } else {
        assert(pv->source_use_mask & (1 << src_id));
        pv->source_use_mask &= ~(1 << src_id);
      }

#ifdef CONFIG_DEVICE_SLEEP
      if (!(pv->source_use_mask & PSOC4_CLOCK_SRC_HF_MASK))
        device_sleep_schedule(dev);
#endif

      dev_cmu_src_update(src, param->flags);
      return 0;

    case DEV_CLOCK_SETUP_SCALER: {
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

    case DEV_CLOCK_SETUP_LINK:
      return 0;

    case DEV_CLOCK_SETUP_UNLINK:
      return 0;

    default:
      return -ENOTSUP;
    }
}

static DEV_USE(psoc4_clock_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_SLEEP
    case DEV_USE_SLEEP: {
      struct device_s *dev = param;
      struct psoc4_clock_private_s *pv = dev->drv_pv;

      if ((pv->source_use_mask & PSOC4_CLOCK_SRC_NOSLEEP_MASK) == 0
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
          && pv->hfclk_sel == SRSS_CLK_SELECT_DIRECT_SEL_IMO
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

static DEV_INIT(psoc4_clock_init);
static DEV_CLEANUP(psoc4_clock_cleanup);

DRIVER_DECLARE(psoc4_clock_drv, DRIVER_FLAGS_EARLY_INIT,
               "PSOC4 Clock", psoc4_clock,
               DRIVER_CMU_METHODS(psoc4_clock));

DRIVER_REGISTER(psoc4_clock_drv);

static DEV_INIT(psoc4_clock_init)
{
  struct psoc4_clock_private_s *pv;
  const void *pclk_src = NULL; // Keep GCC happy

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         SRSS == addr);
  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         PERI == addr);
  assert(device_get_param_blob(dev, "pclk_src", 0, &pclk_src) == 0);

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
  
  for (uint_fast8_t i = 0; i < PSOC4_CLOCK_SRC_COUNT; ++i)
    dev_clock_source_init(dev, &pv->src[i], &psoc4_clock_ep_setup);

  for (uint_fast8_t i = 0; i < PSOC4_CLOCK_PCLK_COUNT; ++i)
    cpu_mem_write_32(PERI + PERI_PCLK_CTL_ADDR(i), pv->pclk_src[i]);

  psoc4_clock_config_read(dev);

  if (dev_cmu_init(&psoc4_clock_drv, dev))
    goto err_mem;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(psoc4_clock_cleanup)
{
  struct psoc4_clock_private_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

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
