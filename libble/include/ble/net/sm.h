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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_SM_H_
#define BLE_SM_H_

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/layer.h>
#include <net/task.h>

#include <ble/security_db.h>
#include <device/class/crypto.h>

#define BLE_LAYER_TYPE_SM NET_LAYER_TYPE('S', 'M', ' ', ' ')

struct ble_sm_handler_s;
struct dev_rng_s;

enum ble_sm_state_e
{
  BLE_SM_IDLE,
  BLE_SM_REQUEST_DONE,
  BLE_SM_STK_DONE,
  BLE_SM_DISTRIBUTION_DONE,
};

/**
 BLE Security manager layer.

 Handles device pairing and bonding.
 */
struct ble_sm_s
{
  struct net_layer_s layer;

  struct dev_rng_s *rng;
  struct device_crypto_s aes_dev;

  const struct ble_sm_handler_s *handler;
  struct ble_peer_s *peer;

  enum ble_sm_state_e pairing_state;
  bool_t security_requested;

  struct ble_addr_s local_addr;

  uint8_t *oob;
  uint16_t passkey;

  uint8_t io_cap;
  bool_t mitm_protection;
  bool_t bonding_enabled;

  struct {
    uint8_t preq[7];
    uint8_t pres[7];
    uint8_t srand[16];
    uint8_t tk[16];
    uint8_t mconf[16];
  };
};

STRUCT_COMPOSE(ble_sm_s, layer);

struct ble_sm_pairing_task_s
{
  struct net_task_s task;

  //  enum ble_sm_pairing_state state;
};

struct ble_peer_s;

struct ble_sm_handler_s
{
  void (*destroyed)(struct ble_sm_s *sm);
};

error_t ble_sm_init(
  struct ble_sm_s *sm,
  const struct ble_sm_handler_s *handler,
  struct net_scheduler_s *scheduler,
  struct ble_peer_s *peer,
  const struct ble_addr_s *local_addr,
  struct dev_rng_s *rng,
  const char *aes_dev);

void ble_sm_cleanup(
  struct ble_sm_s *sm);

#endif
