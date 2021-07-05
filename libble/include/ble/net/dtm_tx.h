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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2020
*/

#ifndef BLE_DTM_TX_H_
#define BLE_DTM_TX_H_

#include <hexo/types.h>
#include <net/task.h>
#include <net/layer.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/dtm.h>

struct net_layer_s;
struct net_scheduler_s;

struct ble_dtm_tx_param_s
{
  // For each event, a packet will be sent on all channels in the channel map, in a row.
  uint64_t channel_map;
  uint32_t interval_ms;
  uint32_t delay_max_ms;
  uint16_t tx_power;
  // PDU size, including header (>= 2)
  uint8_t size;
  enum ble_phy_mode_e phy;
  enum ble_dtm_pattern_e pattern;
};

struct ble_dtm_tx_handler_s
{
  struct net_layer_handler_s base;

  error_t (*params_update)(struct net_layer_s *layer, const struct ble_dtm_tx_param_s *params);
};

STRUCT_COMPOSE(ble_dtm_tx_handler_s, base);

ALWAYS_INLINE
error_t ble_dtm_tx_params_update(struct net_layer_s *layer,
                                     const struct ble_dtm_tx_param_s *params)
{
  const struct ble_dtm_tx_handler_s *handler
    = const_ble_dtm_tx_handler_s_from_base(layer->handler);

  return handler->params_update(layer, params);
}

#endif
