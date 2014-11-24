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
#ifndef BLE_UTIL_DATAPATH_H_
#define BLE_UTIL_DATAPATH_H_

#include <hexo/types.h>
#include <net/layer.h>
#include <net/task.h>
#include <mutek/buffer_pool.h>
#include <device/class/crypto.h>

struct ble_datapath_s;
struct device_crypto_s;

enum ble_datapath_way_e {
  DATA_WAY_RX,
  DATA_WAY_TX,
  DATA_WAY_COUNT,
};

struct ble_datapath_handler_s
{
  void (*pending)(struct ble_datapath_s *data);
  void (*error)(struct ble_datapath_s *data, uint8_t reason);
};

void ble_datapath_packet_push(struct ble_datapath_s *data,
                          enum ble_datapath_way_e way,
                          struct buffer_s *buffer);
void ble_datapath_tx_push(struct ble_datapath_s *data, struct buffer_s *buffer);

#if defined(CONFIG_BLE_CRYPTO)
void ble_datapath_encryption_enable(struct ble_datapath_s *data, enum ble_datapath_way_e way);
void ble_datapath_encryption_disable(struct ble_datapath_s *data, enum ble_datapath_way_e way);
error_t ble_datapath_encryption_setup(struct ble_datapath_s *data,
                                      uint8_t sk[static 16],
                                      uint8_t iv[static 8]);
#endif

struct ble_datapath_s
{
  const struct ble_datapath_handler_s *handler;

  struct {
    buffer_queue_root_t queue;
#if defined(CONFIG_BLE_CRYPTO)
    buffer_queue_root_t crypto_queue;
    struct dev_crypto_context_s ccm_ctx;
    bool_t crypto_enabled;
#endif
  } way[DATA_WAY_COUNT];

  struct kroutine_s pending_kr;

#if defined(CONFIG_BLE_CRYPTO)
  struct buffer_s *ccm_in_packet;
  struct buffer_s *ccm_out_packet;
  struct dev_crypto_rq_s crypto_rq;
  struct device_crypto_s crypto;
  int8_t ccm_retries;
  uint8_t sk[16];
#endif
};

error_t ble_datapath_init(struct ble_datapath_s *data,
                          const struct ble_datapath_handler_s *handler,
                          struct device_crypto_s *crypto);

void ble_datapath_cleanup(struct ble_datapath_s *data);

#endif
