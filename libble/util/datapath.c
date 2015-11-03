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

#include <mutek/printk.h>
#include <ble/util/datapath.h>
#include <ble/protocol/error.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

#if defined(CONFIG_BLE_CRYPTO)
static error_t _ble_datapath_ccm_run(struct ble_datapath_s *data,
                                 struct buffer_s *in,
                                 struct buffer_s *out,
                                 enum ble_datapath_way_e way);

static error_t _ble_datapath_crypto_next(struct ble_datapath_s *data, enum ble_datapath_way_e way);

static KROUTINE_EXEC(ble_datapath_ccm_done)
{
  struct ble_datapath_s *data = KROUTINE_CONTAINER(kr, *data, crypto_rq.rq.kr);
  struct buffer_s *in = data->ccm_in_packet;
  struct buffer_s *out = data->ccm_out_packet;
  enum ble_datapath_way_e way = data->crypto_rq.ctx == &data->way[DATA_WAY_RX].ccm_ctx
    ? DATA_WAY_RX : DATA_WAY_TX;

  if (data->crypto_rq.err) {
    if (data->ccm_retries >= 0) {
      _ble_datapath_ccm_run(data, data->ccm_in_packet, data->ccm_out_packet, way);
      return;
    }

    data->handler->error(data, way == DATA_WAY_RX
                         ? BLE_AUTHENTICATION_FAILURE
                         : BLE_HARDWARE_FAILURE);
    return;
  }

  data->way[way].ccm_state.packet_counter++;

  bool_t empty = buffer_queue_isempty(&data->way[way].queue);

  buffer_queue_pushback(&data->way[way].queue, out);
  buffer_refdec(in);
  buffer_refdec(out);

  data->ccm_in_packet = NULL;
  data->ccm_out_packet = NULL;
  data->ccm_retries = 3;

  _ble_datapath_crypto_next(data, DATA_WAY_RX);
  _ble_datapath_crypto_next(data, DATA_WAY_TX);

  if (empty)
    kroutine_exec(&data->pending_kr);
}

static error_t _ble_datapath_ccm_run(struct ble_datapath_s *data,
                                 struct buffer_s *in,
                                 struct buffer_s *out,
                                 enum ble_datapath_way_e way)
{
  data->ccm_retries--;

  data->ccm_in_packet = in;
  data->ccm_out_packet = out;

  data->crypto_rq.op = DEV_CRYPTO_FINALIZE;
  data->crypto_rq.ctx = &data->way[way].ccm_ctx;
  data->crypto_rq.ad_len = 0;
  data->crypto_rq.ad = 0;
  data->crypto_rq.out = out->data + out->begin;
  data->crypto_rq.in = in->data + in->begin;
  data->crypto_rq.len = in->end - in->begin;

  DEVICE_OP(&data->crypto, request, &data->crypto_rq);

  return 0;
}

static error_t _ble_datapath_crypto_next(struct ble_datapath_s *data, enum ble_datapath_way_e way)
{
  struct buffer_s *in, *out;

  if (data->ccm_in_packet)
    return -EBUSY;

  in = buffer_queue_pop(&data->way[way].crypto_queue);

  if (!in)
    return -ENOENT;
  
  out = buffer_pool_alloc(in->pool);
  out->begin = in->begin;
  out->end = out->begin + in->end - in->begin + (way == DATA_WAY_RX ? -4 : 4);

  return _ble_datapath_ccm_run(data, in, out, way);
}

void ble_datapath_encryption_enable(struct ble_datapath_s *data, enum ble_datapath_way_e way)
{
    
}

void ble_datapath_encryption_disable(struct ble_datapath_s *data, enum ble_datapath_way_e way)
{
}
#endif

static KROUTINE_EXEC(ble_datapath_pending_exec)
{
  struct ble_datapath_s *data = KROUTINE_CONTAINER(kr, *data, pending_kr);

  dprintk("%s\n", __FUNCTION__);

  data->handler->pending(data);
}

void ble_datapath_packet_push(struct ble_datapath_s *data, enum ble_datapath_way_e way, struct buffer_s *packet)
{
#if defined(CONFIG_BLE_CRYPTO)
  if (data->way[way].crypto_enabled) {
    buffer_queue_pushback(&data->way[way].crypto_queue, packet);
    _ble_datapath_crypto_next(data, way);
    return;
  }
#endif

  buffer_queue_pushback(&data->way[way].queue, packet);
  kroutine_exec(&data->pending_kr);
}

error_t ble_datapath_init(struct ble_datapath_s *data,
                          const struct ble_datapath_handler_s *handler,
                          struct device_crypto_s *crypto)
{
  memset(data, 0, sizeof(*data));

  data->handler = handler;

#if defined(CONFIG_BLE_CRYPTO)
  error_t err = device_copy_accessor(&data->crypto, crypto);
  if (err)
    return err;

  struct dev_crypto_info_s crypto_info;
  DEVICE_OP(&data->crypto, info, &crypto_info);

  data->ccm_ctx.state_data = mem_alloc(crypto_info.state_size, mem_scope_sys);
#endif

  kroutine_init(&data->pending_kr, ble_datapath_pending_exec, KROUTINE_INTERRUPTIBLE);

  for (uint8_t way = 0; way < DATA_WAY_COUNT; ++way) {
#if defined(CONFIG_BLE_CRYPTO)
    buffer_queue_init(&data->way[way].crypto_queue);
#endif
    buffer_queue_init(&data->way[way].queue);
  }

  return 0;
}

void ble_datapath_cleanup(struct ble_datapath_s *data)
{
  for (uint8_t way = 0; way < DATA_WAY_COUNT; ++way) {
#if defined(CONFIG_BLE_CRYPTO)
    mem_free(data->state_data);
    buffer_queue_destroy(&data->way[DATA_WAY_TX].crypto_queue);
#endif
    buffer_queue_destroy(&data->way[DATA_WAY_TX].queue);
  }

#if defined(CONFIG_BLE_CRYPTO)
  device_put_accessor(&data->crypto);
#endif
}

#if defined(CONFIG_BLE_CRYPTO)
error_t ble_datapath_encryption_setup(struct ble_datapath_s *data,
                                      uint8_t sk[static 16],
                                      uint8_t iv[static 8])
{
  error_t err;

  data->crypto_rq.op = DEV_CRYPTO_INIT;
  data->crypto_rq.ctx = &data->ccm_ctx;
  data->crypto_rq.iv_ctr = iv;
  data->ccm_ctx.key_data = sk;
  data->ccm_retries = 3;
  data->ccm_ctx.mode = DEV_CRYPTO_MODE_BLE_CCM;
  data->ccm_ctx.key_len = 16;
  data->ccm_ctx.iv_len = 8;
  data->ccm_ctx.auth_len = 2;
  data->ccm_ctx.encrypt_only = 1;

  err = dev_crypto_wait_op(&data->crypto, &data->crypto_rq);
  if (err)
    return err;

  data->way[DATA_WAY_RX].ccm_state.packet_counter = 0;
  data->way[DATA_WAY_RX].ccm_state.sent_by_master = 1;
  data->way[DATA_WAY_TX].ccm_state.packet_counter = 0;
  data->way[DATA_WAY_TX].ccm_state.sent_by_master = 0;

  kroutine_init(&data->crypto_rq.rq.kr, ble_datapath_ccm_done, KROUTINE_INTERRUPTIBLE);

  return 0;
}
#endif
