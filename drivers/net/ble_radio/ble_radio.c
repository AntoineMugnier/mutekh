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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <string.h>

#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>
#include <device/class/ble_radio.h>
#include <device/class/net.h>

#include <mutek/printk.h>

#include "ble_radio_private.h"

static DEV_NET_LAYER_CREATE(ble_net_layer_create)
{
  struct device_s *dev = accessor->dev;
  struct ble_private_s *pv = dev->drv_pv;

  switch (type) {
#if defined(CONFIG_BLE_CENTRAL)
  case BLE_NET_LAYER_MASTER:
    return ble_master_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
#if defined(CONFIG_BLE_PERIPHERAL)
  case BLE_NET_LAYER_ADV:
    return ble_advertiser_create(scheduler, pv, params, delegate, delegate_vtable, layer);
  case BLE_NET_LAYER_SLAVE:
    return ble_slave_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
  default:
    return -ENOTSUP;
  }
}

static DEV_NET_GET_INFO(ble_net_get_info)
{
  struct device_s *dev = accessor->dev;
  struct ble_private_s *pv = dev->drv_pv;
  struct ble_radio_info_s ble_info;

  DEVICE_OP(&pv->radio, get_info, &ble_info);

  memset(info, 0, sizeof(*info));
  info->implemented_layers = 0
#if defined(CONFIG_BLE_CENTRAL)
    | (1 << BLE_NET_LAYER_MASTER)
#endif
#if defined(CONFIG_BLE_PERIPHERAL)
    | (1 << BLE_NET_LAYER_ADV)
    | (1 << BLE_NET_LAYER_SLAVE)
#endif
    ;
  info->prefix_size = ble_info.prefix_size;
  info->mtu = ble_info.mtu;
  info->addr.random_addr = ble_info.address.type == BLE_ADDR_RANDOM;

  memcpy(info->addr.mac, ble_info.address.addr, 6);

  return 0;
}

static DEV_CLEANUP(ble_net_cleanup);
static DEV_INIT(ble_net_init);
#define ble_net_use dev_use_generic

DRIVER_DECLARE(ble_net_drv, 0, "BLE Network", ble_net,
               DRIVER_NET_METHODS(ble_net));

DRIVER_REGISTER(ble_net_drv);

static DEV_CLEANUP(ble_net_cleanup)
{
  struct ble_private_s *pv = dev->drv_pv;

  device_put_accessor(&pv->radio);
  device_put_accessor(&pv->timer);

#if defined(CONFIG_BLE_CRYPTO)
  device_put_accessor(&pv->crypto);
  dev_rng_cleanup(&pv->rng);
#endif

  mem_free(pv);

  return 0;
}

static DEV_INIT(ble_net_init)
{
  struct ble_private_s *pv;
  error_t err;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = device_get_param_dev_accessor(dev, "radio", &pv->radio, DRIVER_CLASS_BLE_RADIO);
  if (err) {
    goto err_pv;
  }

  err = device_get_param_dev_accessor(dev, "radio", &pv->timer, DRIVER_CLASS_TIMER);
  if (err) {
    goto err_close_radio;
  }

#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s seed;
  const char *rng_name, *seed_name;

  err = device_get_param_dev_accessor(dev, "crypto", &pv->crypto, DRIVER_CLASS_CRYPTO);
  if (err) {
    goto err_close_timer;
  }

  err = device_get_param_str(dev, "rng", &rng_name);
  if (err) {
    printk("RNG dep name not found: %d\n", err);
    goto err_close_crypto;
  }

  err = device_get_param_str(dev, "seed", &seed_name);
  if (err) {
    printk("Seed dep name not found: %d\n", err);
    goto err_close_crypto;
  }

  err = dev_rng_init(&pv->rng, rng_name);
  if (err) {
    printk("Error while initing DRBG: %d\n", err);
    goto err_close_crypto;
  }

  err = dev_rng_init(&seed, seed_name);
  if (err) {
    printk("Error while initing RNG: %d\n", err);
    goto err_close_rng;
  }

  err = dev_rng_wait_seed_from_other(&pv->rng, &seed, 16);
  if (err) {
    printk("Error while seeding from RNG: %d\n", err);
    goto err_close_seed;
  }

  dev_rng_cleanup(&seed);
#endif

  dev->drv_pv = pv;
  dev->drv = &ble_net_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#if defined(CONFIG_BLE_CRYPTO)
 err_close_seed:
  dev_rng_cleanup(&seed);
 err_close_rng:
  dev_rng_cleanup(&pv->rng);
 err_close_crypto:
  device_put_accessor(&pv->crypto);
 err_close_timer:
  device_put_accessor(&pv->timer);
#endif
 err_close_radio:
  device_put_accessor(&pv->radio);
 err_pv:
  mem_free(pv);
  return 1;
}
