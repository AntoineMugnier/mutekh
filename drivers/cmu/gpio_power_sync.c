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
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/cmu.h>
#include <device/class/gpio.h>

//#define dprintk printk
#define dprintk(...) do{}while(0)

struct cmu_gpio_power_sync_private_s
{
  struct device_gpio_s gpio_dev;
  gpio_id_t pin_id;
  bool_t is_active;
  bool_t active_up;

  struct dev_clock_src_ep_s src_ep;
};

DRIVER_PV(struct cmu_gpio_power_sync_private_s);

static
bool_t cmu_gpio_power_sync_is_requested(struct cmu_gpio_power_sync_private_s *pv)
{
  return dev_clock_src_is_power_requested(&pv->src_ep);
}

static DEV_CMU_NODE_INFO(cmu_gpio_power_sync_node_info)
{
  struct device_s *dev = accessor->dev;
  struct cmu_gpio_power_sync_private_s *pv = dev->drv_pv;
  enum dev_cmu_node_info_e returned = 0;

  if (node_id > 0)
    return -EINVAL;

  info->name = "Power";
  info->src = &pv->src_ep;
  info->running = pv->is_active;

  returned |= DEV_CMU_INFO_NAME;
  returned |= DEV_CMU_INFO_SRC;
  returned |= DEV_CMU_INFO_RUNNING;

  *mask &= returned;

  return 0;
}

static
void cmu_gpio_power_sync_update(struct cmu_gpio_power_sync_private_s *pv)
{
  bool_t required = cmu_gpio_power_sync_is_requested(pv);
  bool_t active = pv->is_active;

  dprintk("%s %d %d\n", __FUNCTION__, required, active);

  if (active == required)
    return;

  dev_gpio_out(&pv->gpio_dev, pv->pin_id,
               required ? pv->active_up : !pv->active_up);

  pv->is_active = required;
}

static DEV_CLOCK_SRC_SETUP(cmu_gpio_power_sync_ep_setup)
{
  struct device_s *dev = src->dev;
  struct cmu_gpio_power_sync_private_s *pv = dev->drv_pv;

  if (src != &pv->src_ep)
    return -ENOENT;

  switch (op) {
  case DEV_CLOCK_SRC_SETUP_LINK:
    dprintk("%s link\n", __FUNCTION__);
    return 0;

  case DEV_CLOCK_SRC_SETUP_UNLINK:
    dprintk("%s unlink\n", __FUNCTION__);
    return 0;

  case DEV_CLOCK_SRC_SETUP_GATES:
    dprintk("%s setup gates %d\n", __FUNCTION__, !!(param->flags & DEV_CLOCK_EP_POWER));
#if defined(CONFIG_DEVICE_SLEEP)
    if (!(param->flags & DEV_CLOCK_EP_POWER)) {
      dprintk("%s scheduling sleep\n", __FUNCTION__);
      device_sleep_schedule(dev);
      return -EAGAIN;
    }
#endif

    dprintk("%s update\n", __FUNCTION__);
    cmu_gpio_power_sync_update(pv);
    dev_cmu_src_update_sync(&pv->src_ep, pv->is_active & DEV_CLOCK_EP_POWER);

    return 0;

  default:
    return -ENOTSUP;
  }

  return 0;
}

static DEV_USE(cmu_gpio_power_sync_use)
{
  switch (op) {
#if defined(CONFIG_DEVICE_SLEEP)
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct cmu_gpio_power_sync_private_s *pv = dev->drv_pv;

    dprintk("%s sleep\n", __FUNCTION__);
    
    cmu_gpio_power_sync_update(pv);
    dev_cmu_src_update_async(&pv->src_ep, pv->is_active ? DEV_CLOCK_EP_POWER : 0);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_CMU_APP_CONFIGID_SET(cmu_gpio_power_sync_app_configid_set)
{
  struct device_s *dev = accessor->dev;
  struct cmu_gpio_power_sync_private_s *pv = dev->drv_pv;

  (void)pv;

  return 0;
}

static DEV_INIT(cmu_gpio_power_sync_init)
{
  struct cmu_gpio_power_sync_private_s *pv;
  uintptr_t tmp;
  gpio_id_t pin;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  err = device_get_param_uint(dev, "active", &tmp);
  pv->active_up = err || tmp;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio_dev.base, DRIVER_CLASS_GPIO);
  if (err)
    goto free_pv;

  err = device_res_gpio_map(dev, "power:1", &pin, NULL);
  if (err)
    goto free_pv;

  dev_clock_source_init(dev, &pv->src_ep, &cmu_gpio_power_sync_ep_setup);

  dev_gpio_out(&pv->gpio_dev, pin, !pv->active_up);

  gpio_width_t width = 1;
  err = device_gpio_map_set_mode(&pv->gpio_dev, &pin, &width, 1, DEV_PIN_PUSHPULL);
  if (err)
    goto free_pv;

  pv->pin_id = pin;
  cmu_gpio_power_sync_update(pv);
  dev_cmu_src_update_async(&pv->src_ep, 0);

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(cmu_gpio_power_sync_cleanup)
{
  struct cmu_gpio_power_sync_private_s *pv = dev->drv_pv;

  if (pv->is_active)
    return -EAGAIN;

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(cmu_gpio_power_sync_drv, 0, "GPIO Power Sync", cmu_gpio_power_sync,
               DRIVER_CMU_METHODS(cmu_gpio_power_sync));

DRIVER_REGISTER(cmu_gpio_power_sync_drv);
