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

#include <arch/psoc4/bless.h>
#include <arch/psoc4/blell.h>
#include <arch/psoc4/blerd.h>
#include <arch/psoc4/sflash.h>
#include <arch/psoc4/variant.h>

#define BLESS PSOC4_BLESS_ADDR
#define BLERD PSOC4_BLERD_ADDR
#define BLELL PSOC4_BLELL_ADDR

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#define SINK_LFCLK 0

struct psoc4_ble_private_s
{
  struct dev_clock_src_ep_s src[PSOC4_BLE_CLK_SRC_COUNT];
  struct dev_clock_sink_ep_s sink[PSOC4_BLE_CLK_SINK_COUNT];

  struct dev_freq_s eco_freq;
  struct dev_freq_s wco_freq;

  struct dev_freq_s lfclk_freq;
  uint8_t eco_ll_div;
  uint8_t eco_clk_div;
  uint8_t notify_mask;
};

static DEV_CMU_CONFIG_OSC(psoc4_ble_config_osc)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  if (freq->denom != 1)
    return -EINVAL;

  switch (node_id) {
  case PSOC4_BLE_CLK_OSC_ECO: {
    if (freq->num != 24000000)
      return -EINVAL;

    pv->eco_freq = *freq;
    return 0;
  }

  case PSOC4_BLE_CLK_SRC_WCO:
    if (freq->num != 32768)
      return -EINVAL;

    pv->wco_freq = *freq;
    return 0;

  default:
    return -ENOENT;
  }

  return -ENOENT;
}

static DEV_CMU_CONFIG_MUX(psoc4_ble_config_mux)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;

  switch (node_id) {
  case PSOC4_BLE_CLK_SRC_ECO: {
    uint32_t div = ratio->denom;

    if (ratio->num > 1)
      div /= ratio->num;

    if (div & (div - 1))
      return -EINVAL;

    if (div > 8)
      return -EINVAL;

    if (parent_id != PSOC4_BLE_CLK_OSC_ECO)
      return -EINVAL;

    pv->eco_clk_div = __builtin_ctz(div);
    return 0;
  }

  default:
    return -ENOENT;
  }

  return 0;
}

static void psoc4_ble_config_read(struct device_s *dev)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t xtal_config = cpu_mem_read_32(BLESS + BLESS_XTAL_CLK_DIV_CONFIG_ADDR);

  pv->eco_clk_div = BLESS_XTAL_CLK_DIV_CONFIG_SYSCLK_DIV_GET(xtal_config);
  pv->eco_ll_div = BLESS_XTAL_CLK_DIV_CONFIG_LLCLK_DIV_GET(xtal_config);
}

static DEV_CMU_ROLLBACK(psoc4_ble_rollback)
{
  struct device_s *dev = accessor->dev;

  psoc4_ble_config_read(dev);

  return 0;
}

static DEV_CMU_COMMIT(psoc4_ble_commit)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t tmp;
  struct dev_clock_notify_s notif = {
    .freq = pv->eco_freq,
  };

  tmp = 0;
  BLESS_XTAL_CLK_DIV_CONFIG_LLCLK_DIV_SETVAL(tmp, pv->eco_ll_div);
  BLESS_XTAL_CLK_DIV_CONFIG_SYSCLK_DIV_SETVAL(tmp, pv->eco_clk_div);
  cpu_mem_write_32(BLESS + BLESS_XTAL_CLK_DIV_CONFIG_ADDR, tmp);

  notif.freq.denom <<= pv->eco_clk_div;

  if (pv->notify_mask & (1 << PSOC4_BLE_CLK_SRC_ECO))
    dev_cmu_src_notify(&pv->src[PSOC4_BLE_CLK_SRC_ECO], &notif);

  return 0;
}

static DEV_CMU_NODE_INFO(psoc4_ble_node_info)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t returned = 0
    | DEV_CMU_INFO_FREQ
    | DEV_CMU_INFO_NAME
    | DEV_CMU_INFO_RUNNING
    | DEV_CMU_INFO_ACCURACY
    ;

  static const char *const node_name[] = {
    [PSOC4_BLE_CLK_SRC_ECO] = "ECO_CLK",
    [PSOC4_BLE_CLK_SRC_WCO] = "WCO",
    [PSOC4_BLE_CLK_OSC_ECO] = "ECO",
    [PSOC4_BLE_CLK_SINK_LFCLK] = "LFCLK",
  };

  if (node_id >= PSOC4_BLE_CLK_NODE_COUNT)
    return -EINVAL;

  if (node_id < PSOC4_BLE_CLK_SRC_COUNT) {
    returned |= DEV_CMU_INFO_SRC;
    info->src = &pv->src[node_id];
  }

  info->name = node_name[node_id];

  switch ((enum psoc4_ble_clock_e)node_id) {
  case PSOC4_BLE_CLK_SRC_ECO:
    info->freq = pv->eco_freq;
    info->running = !!(cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR)
                       & BLERD_DBUS_XTAL_ENABLE);
    returned |= DEV_CMU_INFO_PARENT;
    info->parent_id = PSOC4_BLE_CLK_OSC_ECO;
    break;

  case PSOC4_BLE_CLK_SRC_WCO:
    info->freq = pv->wco_freq;
    info->running = !!(cpu_mem_read_32(BLESS + BLESS_WCO_STATUS_ADDR)
                       & BLESS_WCO_STATUS_OUT_BLNK_A);
    break;

  case PSOC4_BLE_CLK_OSC_ECO:
    info->freq = pv->eco_freq;
    info->freq.denom <<= pv->eco_clk_div;
    info->running = !!(cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR)
                       & BLERD_DBUS_XTAL_ENABLE);
    break;

  case PSOC4_BLE_CLK_SINK_LFCLK:
    info->freq = pv->lfclk_freq;
    returned |= DEV_CMU_INFO_SINK;
    info->sink = &pv->sink[SINK_LFCLK];
    break;

  default:
    return -EINVAL;
  }

  *mask &= returned;

  if (info->freq.denom > 1 && *mask & DEV_CMU_INFO_FREQ) {
    uint64_t g = gcd64(info->freq.num, info->freq.denom);
    info->freq.num /= g;
    info->freq.denom /= g;
  }

  return 0;
}

static void psoc4_ble_eco_start(struct device_s *dev, bool_t sync)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t dbus = cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR);
  uint32_t tmp;

  dev_clock_sink_gate(&pv->sink[SINK_LFCLK], DEV_CLOCK_EP_CLOCK);

  // Enable RF
  tmp = cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR);
  tmp |= BLESS_RF_CONFIG_RF_ENABLE;
  cpu_mem_write_32(BLESS + BLESS_RF_CONFIG_ADDR, tmp);

  // Clear irq bit
  cpu_mem_write_32(BLESS + BLESS_LL_DSM_INTR_STAT_ADDR,
                   BLESS_LL_DSM_INTR_STAT_XTAL_ON_INTR);

  dbus |= BLERD_DBUS_XTAL_ENABLE;
  cpu_mem_write_32(BLERD + BLERD_DBUS_ADDR, dbus);

  if (!sync)
    return;

  while (!(cpu_mem_read_32(BLESS + BLESS_LL_DSM_INTR_STAT_ADDR)
           & BLESS_LL_DSM_INTR_STAT_XTAL_ON_INTR))
    ;
}

static void psoc4_ble_eco_stop(struct device_s *dev)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;

  dev_clock_sink_gate(&pv->sink[SINK_LFCLK], DEV_CLOCK_EP_NONE);

  device_sleep_schedule(dev);
}

static DEV_CLOCK_SRC_SETUP(psoc4_ble_ep_setup)
{
  struct device_s *dev = src->dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint_fast8_t src_id = src - pv->src;

  if (src_id >= PSOC4_BLE_CLK_SRC_COUNT)
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
      case PSOC4_BLE_CLK_SRC_ECO: {
        if (param->flags & DEV_CLOCK_EP_ANY) {
          psoc4_ble_eco_start(dev, !!(param->flags & DEV_CLOCK_EP_SINK_SYNC));
        } else {
          psoc4_ble_eco_stop(dev);
        }

        dev_cmu_src_update(src, param->flags);

        break;
      }

      case PSOC4_BLE_CLK_SRC_WCO: {
        uint32_t wco_config = cpu_mem_read_32(BLESS + BLESS_WCO_CONFIG_ADDR);
        uint32_t wco_status = cpu_mem_read_32(BLESS + BLESS_WCO_STATUS_ADDR);

        if ((param->flags & DEV_CLOCK_EP_ANY) && !(wco_status & BLESS_WCO_STATUS_OUT_BLNK_A)) {
          wco_config |= BLESS_WCO_CONFIG_ENABLE;
          cpu_mem_write_32(BLESS + BLESS_WCO_CONFIG_ADDR, wco_config);

          while ((param->flags & DEV_CLOCK_EP_SINK_SYNC)
                 && !(cpu_mem_read_32(BLESS + BLESS_WCO_STATUS_ADDR)
                      & BLESS_WCO_STATUS_OUT_BLNK_A))
            ;
        } else if (!(param->flags & DEV_CLOCK_EP_ANY) && (wco_config & BLESS_WCO_CONFIG_ENABLE)) {
          wco_config &= ~BLESS_WCO_CONFIG_ENABLE;
          cpu_mem_write_32(BLESS + BLESS_WCO_CONFIG_ADDR, wco_config);

          device_sleep_schedule(dev);
        }

        dev_cmu_src_update(src, param->flags);

        break;

      return 0;
      }

      default:
        return -EINVAL;
      }

    case DEV_CLOCK_SETUP_LINK:
      return 0;

    case DEV_CLOCK_SETUP_UNLINK:
      return 0;

    default:
      return -ENOTSUP;
    }
}

static DEV_USE(psoc4_ble_use)
{
  switch (op) {
#ifdef CONFIG_DEVICE_SLEEP
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct psoc4_ble_private_s *pv = dev->drv_pv;

    (void)pv;

    return 0;
  }
#endif

  case DEV_USE_CLOCK_NOTIFY: {
    struct dev_clock_notify_s *notify = param;
    struct dev_clock_sink_ep_s *sink = notify->sink;
    struct device_s *dev = sink->dev;
    struct psoc4_ble_private_s *pv = dev->drv_pv;

    assert(sink == &pv->sink[SINK_LFCLK]);

    printk("PSoC4 BLE LFCLK notify %d/%d\n",
           (uint32_t)notify->freq.num, (uint32_t)notify->freq.denom);

    pv->lfclk_freq = notify->freq;

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

const struct driver_s psoc4_ble_drv;

static DEV_INIT(psoc4_ble_init)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  error_t err;

  if (!pv) {
    pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
    if (!pv)
      return -ENOMEM;

    memset(pv, 0, sizeof (*pv));
    dev->drv_pv = pv;
  }

  if (!(dev->init_mask & (1 << 0))) {
    psoc4_ble_config_read(dev);

    err = dev_cmu_init(&psoc4_ble_drv, dev);
    if (err) {
      if (err != -EAGAIN)
        goto err_mem;

      return err;
    }

    for (uint_fast8_t i = 0; i < PSOC4_BLE_CLK_SRC_COUNT; ++i)
      dev_clock_source_init(dev, &pv->src[i], &psoc4_ble_ep_setup);

    device_init_enable_api(dev, 0);
  }

  if (cl_missing & (1 << DRIVER_CLASS_CMU))
    return -EAGAIN;

  if (!pv->sink[SINK_LFCLK].src) {
    err = dev_drv_clock_init(dev, &pv->sink[SINK_LFCLK], PSOC4_BLE_CLK_SINK_LFCLK,
                             DEV_CLOCK_EP_VARFREQ, &pv->lfclk_freq);
    if (err) {
      printk("%s: LFCLK not ready yet\n", __FUNCTION__);
      return -EAGAIN;
    }
  }

  return 0;

 err_mem:
  dev->drv_pv = NULL;
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(psoc4_ble_cleanup)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(psoc4_ble_drv, DRIVER_FLAGS_NO_DEPEND | DRIVER_FLAGS_RETRY_INIT,
               "PSoC4 BLE", psoc4_ble,
               DRIVER_CMU_METHODS(psoc4_ble));

DRIVER_REGISTER(psoc4_ble_drv);
