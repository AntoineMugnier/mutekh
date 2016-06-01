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
#include <device/class/cmu.h>

#include <arch/psoc4/peri.h>
#include <arch/psoc4/variant.h>

#define PERI PSOC4_PERI_ADDR

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

DRIVER_PV(struct psoc4_peri_private_s
{
  struct dev_clock_sink_ep_s hfclk;
  struct dev_clock_src_ep_s src[PSOC4_PERI_SRC_COUNT];
  struct dev_freq_s hfclk_freq;
  const uint8_t *pclk_src;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  uint32_t notify_mask;
#endif
  uint32_t source_use_mask;
});

static DEV_CMU_NODE_INFO(psoc4_peri_node_info)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_peri_private_s *pv = dev->drv_pv;
  uint32_t returned = 0
    | DEV_CMU_INFO_FREQ
    | DEV_CMU_INFO_NAME
    | DEV_CMU_INFO_RUNNING
    | DEV_CMU_INFO_ACCURACY
    ;

  static const char *const node_name[] = {
    [PSOC4_PERI_SRC_0] = "IMO SS",
    [PSOC4_PERI_SRC_1] = "SCB0",
    [PSOC4_PERI_SRC_2] = "SCB1",
    [PSOC4_PERI_SRC_3] = "PUMP",
    [PSOC4_PERI_SRC_4] = "CSD0",
    [PSOC4_PERI_SRC_5] = "CSD1",
    [PSOC4_PERI_SRC_6] = "SAR",
    [PSOC4_PERI_SRC_7] = "TCPWM0",
    [PSOC4_PERI_SRC_8] = "TCPWM1",
    [PSOC4_PERI_SRC_9] = "TCPWM2",
    [PSOC4_PERI_SRC_10] = "TCPWM3",
    [PSOC4_PERI_SRC_11] = "Peri 11",
    [PSOC4_PERI_SRC_12] = "Peri 12",
    [PSOC4_PERI_SRC_13] = "Peri 13",
    [PSOC4_PERI_SRC_14] = "Peri 14",
    [PSOC4_PERI_SRC_15] = "LCD",
    [PSOC4_PERI_SINK_HFCLK] = "HFCLK",
  };

  if (node_id > PSOC4_PERI_NODE_COUNT)
    return -EINVAL;

  info->name = node_name[node_id];
  info->freq = pv->hfclk_freq;

  switch ((enum psoc4_peri_node_e)node_id) {
  case PSOC4_PERI_SRC_0 ... PSOC4_PERI_SRC_15: {
    uint32_t id = node_id - PSOC4_PERI_SRC_0;
    uint32_t div = pv->pclk_src[id];
    uint32_t div_ctl = cpu_mem_read_32(PERI + PERI_DIV_CTL_ADDR(div));

    returned |= DEV_CMU_INFO_SRC;
    returned |= DEV_CMU_INFO_PARENT;
    info->src = &pv->src[node_id];
    info->parent_id = PSOC4_PERI_SINK_HFCLK;
    info->running = (div_ctl & PERI_DIV_CTL_EN) && (pv->hfclk.src->flags & DEV_CLOCK_EP_CLOCK);

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

  case PSOC4_PERI_SINK_HFCLK:
    returned |= DEV_CMU_INFO_SINK;
    info->sink = &pv->hfclk;
    info->running = !!(pv->hfclk.src->flags & DEV_CLOCK_EP_CLOCK);
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

static bool_t psoc4_pclk_is_enabled(struct device_s *dev,
                                    uint_fast8_t pclk_id)
{
  struct psoc4_peri_private_s *pv = dev->drv_pv;

  uint32_t div_ctl = cpu_mem_read_32(PERI + PERI_DIV_CTL_ADDR(pv->pclk_src[pclk_id]));

  return !!(div_ctl & PERI_DIV_CTL_EN);
}

static void psoc4_pclk_disable(struct device_s *dev,
                               uint_fast8_t pclk_id)
{
  struct psoc4_peri_private_s *pv = dev->drv_pv;

  dprintk("%s %d\n", __FUNCTION__, pclk_id);

  assert(psoc4_pclk_is_enabled(dev, pclk_id));

  cpu_mem_write_32(PERI + PERI_DIV_CMD_ADDR, 0
                   | PERI_DIV_CMD_SEL(pv->pclk_src[pclk_id])
                   | PERI_DIV_CMD_PA_SEL(PSOC4_PERI_DIV_NONE)
                   | PERI_DIV_CMD_DISABLE);

  while (cpu_mem_read_32(PERI + PERI_DIV_CMD_ADDR) & PERI_DIV_CMD_DISABLE)
    ;
}

static void psoc4_pclk_enable(struct device_s *dev,
                               uint_fast8_t pclk_id)
{
  struct psoc4_peri_private_s *pv = dev->drv_pv;

  dprintk("%s %d\n", __FUNCTION__, pclk_id);

  assert(!psoc4_pclk_is_enabled(dev, pclk_id));

  cpu_mem_write_32(PERI + PERI_DIV_CMD_ADDR, 0
                   | PERI_DIV_CMD_SEL(pv->pclk_src[pclk_id])
                   | PERI_DIV_CMD_PA_SEL(PSOC4_PERI_DIV_NONE)
                   | PERI_DIV_CMD_ENABLE);

  while (cpu_mem_read_32(PERI + PERI_DIV_CMD_ADDR) & PERI_DIV_CMD_ENABLE)
    ;
}

static void psoc4_pclk_div_set(struct device_s *dev,
                               uint_fast8_t pclk_id,
                               uint32_t div)
{
  struct psoc4_peri_private_s *pv = dev->drv_pv;

  assert(pv->pclk_src[pclk_id] != PSOC4_PERI_DIV_NONE);
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

static DEV_CLOCK_SRC_SETUP(psoc4_peri_ep_setup)
{
  struct device_s *dev = src->dev;
  struct psoc4_peri_private_s *pv = dev->drv_pv;
  error_t err;
  uint_fast8_t src_id = src - pv->src;
  if (src_id >= PSOC4_PERI_SRC_COUNT)
    return -EINVAL;

  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_CLOCK_SRC_SETUP_NOTIFY:
      pv->notify_mask |= bit(src_id);
      return 0;

    case DEV_CLOCK_SRC_SETUP_NONOTIFY:
      pv->notify_mask &= ~bit(src_id);
      return 0;
#endif

#if defined(CONFIG_DEVICE_CLOCK_GATING)
    case DEV_CLOCK_SRC_SETUP_GATES:
      dprintk("%s gates src %d; %x\n",
             __FUNCTION__, src_id, param->flags);

      if (!psoc4_pclk_is_enabled(dev, src_id)
          && (param->flags & DEV_CLOCK_EP_CLOCK)) {
        err = dev_clock_sink_gate(&pv->hfclk, DEV_CLOCK_EP_CLOCK);

        psoc4_pclk_enable(dev, src_id);
        pv->source_use_mask |= bit(src_id);

        if (err == 0)
          dev_cmu_src_update_sync(src, param->flags);

        return err;
      }

      if (psoc4_pclk_is_enabled(dev, src_id)
          && !(param->flags & DEV_CLOCK_EP_CLOCK)) {
        psoc4_pclk_disable(dev, src_id);
        pv->source_use_mask &= ~bit(src_id);

        dev_cmu_src_update_sync(src, param->flags);

# ifdef CONFIG_DEVICE_SLEEP
        if (!pv->source_use_mask)
          device_sleep_schedule(dev);
# endif

        return 0;
      }
      return 0;
#endif

    case DEV_CLOCK_SRC_SETUP_SCALER: {
      if (src_id >= PSOC4_PERI_SRC_COUNT)
        return -EINVAL;

      uint8_t div_id = pv->pclk_src[src_id];
      uint32_t ratio;
      if (param->scale.num > 1)
        ratio = ((uint64_t)param->scale.denom * 32) / param->scale.num;
      else
        ratio = param->scale.denom * 32;

      dprintk("%s PCLK %d div %d.%d ratio %d\n", __FUNCTION__, src_id,
             PSOC4_PERI_DIV_TYPE(div_id), PSOC4_PERI_DIV_NO(div_id), ratio);

      if (div_id == PSOC4_PERI_DIV_NONE)
        return -ENOENT;

      if (ratio & ~div_mask[PSOC4_PERI_DIV_TYPE(div_id)])
        return -ENOTSUP;

      if (ratio < 32)
        return -EINVAL;

      psoc4_pclk_div_set(dev, src_id, ratio);

      return 0;
    }

    case DEV_CLOCK_SRC_SETUP_LINK:
      if (param->sink->flags & DEV_CLOCK_EP_GATING_SYNC)
        return -ENOTSUP;
      return 0;

    case DEV_CLOCK_SRC_SETUP_UNLINK:
      return 0;

    default:
      return -ENOTSUP;
    }
}

static DEV_USE(psoc4_peri_use)
{
  switch (op) {
#if defined(CONFIG_DEVICE_SLEEP) && defined(CONFIG_DEVICE_CLOCK_GATING)
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct psoc4_peri_private_s *pv = dev->drv_pv;

    if (!pv->source_use_mask)
      dev_clock_sink_gate(&pv->hfclk, 0);

    return 0;
  }
#endif

#if defined(CONFIG_DEVICE_CLOCK_GATING)
  case DEV_USE_CLOCK_SINK_GATE_DONE: {
    struct dev_clock_sink_ep_s *sink = param;
    struct device_s *dev = sink->dev;
    struct psoc4_peri_private_s *pv = dev->drv_pv;

    for (uint_fast8_t i = 0; i < PSOC4_PERI_SRC_COUNT; ++i)
      dev_cmu_src_update_async(&pv->src[i], sink->src->flags & DEV_CLOCK_EP_ANY);

    return 0;
  }
#endif

#if defined(CONFIG_DEVICE_CLOCK_VARFREQ)
  case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
    struct dev_clock_notify_s *notify = param;
    struct dev_clock_sink_ep_s *sink = notify->sink;
    struct device_s *dev = sink->dev;
    struct psoc4_peri_private_s *pv = dev->drv_pv;
    struct dev_clock_notify_s notify2;

    notify2.freq = notify->freq;

    for (uint_fast8_t i = 0; i < PSOC4_PERI_SRC_COUNT; ++i)
      dev_cmu_src_notify(&pv->src[i], &notify2);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(psoc4_peri_init)
{
  struct psoc4_peri_private_s *pv = dev->drv_pv;
  const void *pclk_src = NULL;
  error_t err = -1;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         PERI == addr);

  err = device_get_param_blob(dev, "pclk_src", 0, &pclk_src);
  if (err)
    return err;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof (*pv));
  dev->drv_pv = pv;
  pv->pclk_src = pclk_src;

  for (uint_fast8_t i = 0; i < PSOC4_PERI_SRC_COUNT; ++i) {
    dev_clock_source_init(dev, &pv->src[i], &psoc4_peri_ep_setup);
    cpu_mem_write_32(PERI + PERI_PCLK_CTL_ADDR(i), pv->pclk_src[i]);
  }
  
  return 0;
}

static DEV_CLEANUP(psoc4_peri_cleanup)
{
  struct psoc4_peri_private_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

#define psoc4_peri_app_configid_set (dev_cmu_app_configid_set_t*)dev_driver_notsup_fcn

DRIVER_DECLARE(psoc4_peri_drv, 0, "PSoC4 Peri", psoc4_peri,
               DRIVER_CMU_METHODS(psoc4_peri));

DRIVER_REGISTER(psoc4_peri_drv);
