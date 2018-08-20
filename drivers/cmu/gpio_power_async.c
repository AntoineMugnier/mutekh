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

struct cmu_gpio_power_async_private_s
{
  struct device_gpio_s gpio_dev;
  struct dev_gpio_rq_s gpio_rq;
  bool_t is_active;
  bool_t active_up;

  struct dev_clock_src_ep_s src_ep;
};

DRIVER_PV(struct cmu_gpio_power_async_private_s);

static
bool_t cmu_gpio_power_async_is_requested(struct cmu_gpio_power_async_private_s *pv)
{
  return dev_clock_src_is_power_requested(&pv->src_ep);
}

static
bool_t cmu_gpio_power_async_is_gpio_busy(struct cmu_gpio_power_async_private_s *pv)
{
  return !!pv->gpio_rq.pvdata;
}

static DEV_CMU_NODE_INFO(cmu_gpio_power_async_node_info)
{
  struct device_s *dev = accessor->dev;
  struct cmu_gpio_power_async_private_s *pv = dev->drv_pv;
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
error_t cmu_gpio_power_async_update(struct cmu_gpio_power_async_private_s *pv)
{
  assert(!cmu_gpio_power_async_is_gpio_busy(pv));

  bool_t required = cmu_gpio_power_async_is_requested(pv);
  bool_t active = pv->is_active;

  dprintk("%s %d %d\n", __FUNCTION__, required, active);

  if (active == required)
    return 0;

  dprintk("%s using async API\n", __FUNCTION__);

  if (required ? pv->active_up : !pv->active_up)
    pv->gpio_rq.output.set_mask = pv->gpio_rq.output.clear_mask = dev_gpio_mask1;
  else
    pv->gpio_rq.output.set_mask = pv->gpio_rq.output.clear_mask = dev_gpio_mask0;

  pv->gpio_rq.pvdata = pv->src_ep.dev;
  pv->gpio_rq.type = DEV_GPIO_SET_OUTPUT;

  DEVICE_OP(&pv->gpio_dev, request, &pv->gpio_rq);

  return -EAGAIN;
}

static KROUTINE_EXEC(cmu_gpio_power_async_done)
{
  struct cmu_gpio_power_async_private_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = pv->gpio_rq.pvdata;

  dprintk("%s async API done\n", __FUNCTION__);

  LOCK_SPIN_IRQ(&dev->lock);

  pv->gpio_rq.pvdata = NULL;

  bool_t cur_value = (pv->gpio_rq.output.set_mask == dev_gpio_mask1);
  bool_t active = pv->active_up == cur_value;

  if (pv->gpio_rq.type == DEV_GPIO_MODE)
    active = !cmu_gpio_power_async_is_requested(pv);

  pv->is_active = active;

  if (cmu_gpio_power_async_update(pv) == 0)
    dev_cmu_src_update_async(&pv->src_ep, active ? DEV_CLOCK_EP_POWER : 0);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_CLOCK_SRC_SETUP(cmu_gpio_power_async_ep_setup)
{
  struct device_s *dev = src->dev;
  struct cmu_gpio_power_async_private_s *pv = dev->drv_pv;
  error_t err;

  if (src != &pv->src_ep)
    return -ENOENT;

  switch (op) {
  case DEV_CLOCK_SRC_SETUP_LINK:
    dprintk("%s link\n", __FUNCTION__);
    if (param->sink->flags & DEV_CLOCK_EP_GATING_SYNC)
      return -ENOTSUP;
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

    if (cmu_gpio_power_async_is_gpio_busy(pv)) {
      dprintk("%s busy\n", __FUNCTION__);
      return -EAGAIN;
    }

    //LOCK_SPIN_IRQ(&dev->lock);
    err = cmu_gpio_power_async_update(pv);
    dprintk("%s update %d\n", __FUNCTION__, err);
    if (err == 0)
      dev_cmu_src_update_sync(&pv->src_ep, pv->is_active & DEV_CLOCK_EP_POWER);
    //LOCK_RELEASE_IRQ(&dev->lock);

    return err;

  default:
    return -ENOTSUP;
  }

  return 0;
}

static DEV_USE(cmu_gpio_power_async_use)
{
  switch (op) {
#if defined(CONFIG_DEVICE_SLEEP)
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct cmu_gpio_power_async_private_s *pv = dev->drv_pv;

    dprintk("%s sleep\n", __FUNCTION__);
    
    //LOCK_SPIN_IRQ(&dev->lock);
    if (cmu_gpio_power_async_update(pv) == 0)
      dev_cmu_src_update_async(&pv->src_ep, pv->is_active ? DEV_CLOCK_EP_POWER : 0);
    //LOCK_RELEASE_IRQ(&dev->lock);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_CMU_APP_CONFIGID_SET(cmu_gpio_power_async_app_configid_set)
{
  struct device_s *dev = accessor->dev;
  struct cmu_gpio_power_async_private_s *pv = dev->drv_pv;

  (void)pv;

  return 0;
}

static DEV_INIT(cmu_gpio_power_async_init)
{
  struct cmu_gpio_power_async_private_s *pv;
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

  dev_clock_source_init(dev, &pv->src_ep, &cmu_gpio_power_async_ep_setup);

  dev_gpio_rq_init(&pv->gpio_rq, cmu_gpio_power_async_done);

  pv->gpio_rq.type = DEV_GPIO_MODE;
  pv->gpio_rq.io_first
    = pv->gpio_rq.io_last
    = pin;

  pv->gpio_rq.mode.mask = dev_gpio_mask1;
  pv->gpio_rq.mode.mode = DEV_PIN_PUSHPULL;
  pv->gpio_rq.pvdata = pv->src_ep.dev;

  DEVICE_OP(&pv->gpio_dev, request, &pv->gpio_rq);

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(cmu_gpio_power_async_cleanup)
{
  struct cmu_gpio_power_async_private_s *pv = dev->drv_pv;

  if (cmu_gpio_power_async_is_gpio_busy(pv) || pv->is_active)
    return -EAGAIN;

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(cmu_gpio_power_async_drv, 0, "GPIO Power", cmu_gpio_power_async,
               DRIVER_CMU_METHODS(cmu_gpio_power_async));

DRIVER_REGISTER(cmu_gpio_power_async_drv);
