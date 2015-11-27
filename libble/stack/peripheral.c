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

#include <ble/stack/peripheral.h>
#include <ble/protocol/l2cap.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>

#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>

#include <ble/net/adv.h>
#include <ble/net/phy.h>
#include <ble/net/gatt.h>
#include <ble/net/att.h>
#include <ble/net/link.h>
#include <ble/net/llcp.h>

#include <ble/net/layer_id.h>

#include <mutek/printk.h>

#include <ble/net/generic.h>

static const struct ble_phy_delegate_vtable_s peri_slave_vtable;
#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s sm_delegate_vtable;
#endif
static const struct ble_advertiser_delegate_vtable_s peri_adv_vtable;
static const struct ble_llcp_delegate_vtable_s peri_llcp_vtable;
static error_t adv_start(struct ble_peripheral_s *peri);

struct dev_rng_s;

static void peri_state_update(struct ble_peripheral_s *peri)
{
  enum ble_peripheral_state_e state = BLE_PERIPHERAL_IDLE;

  if (peri->slave) {
    state = BLE_PERIPHERAL_CONNECTED;
  } else if (peri->adv) {
    if (peri->mode & BLE_PERIPHERAL_PAIRABLE)
      state = BLE_PERIPHERAL_PAIRING;
    else
      state = BLE_PERIPHERAL_ADVERTISING;
  }

  if (state == peri->last_state)
    return;

  peri->last_state = state;

  printk("Peripheral state now %d\n", state);

  peri->handler->state_changed(peri, state);
}

static void peri_conn_drop(struct ble_peripheral_s *peri, uint8_t reason)
{
  if (!peri->slave)
    return;

  if (peri->llcp && !reason)
    ble_llcp_connection_close(peri->llcp);

  net_layer_refdec(peri->slave);
  peri->slave = NULL;
  peri->llcp = NULL;

  peri->handler->connection_closed(peri, reason);

  peri_state_update(peri);
  adv_start(peri);
}

static void conn_connection_lost(void *delegate, struct net_layer_s *layer,
                                 uint8_t reason)
{
  struct ble_peripheral_s *peri = delegate;

  printk("Peripheral connection dropped: %d\n", reason);

  if (layer != peri->slave)
    return;

  peri_conn_drop(peri, reason);
}

static void peri_llcp_closed(void *delegate, struct net_layer_s *layer,
                             uint8_t reason)
{
  struct ble_peripheral_s *peri = delegate;

  if (layer != peri->llcp)
    return;

  peri->llcp = NULL;

  peri_conn_drop(peri, reason);
}

static void peri_llcp_destroyed(void *delegate, struct net_layer_s *layer)
{
  printk("LLCP layer released\n");
}

static void peri_slave_destroyed(void *delegate, struct net_layer_s *layer)
{
  printk("Slave layer released\n");
}

#if defined(CONFIG_BLE_CRYPTO)
static
void peri_pairing_requested(void *delegate, struct net_layer_s *layer,
                            bool_t bonding)
{
  struct ble_peripheral_s *peri = delegate;

  printk("Slave pairing requested\n");

  peri->handler->pairing_requested(peri, bonding);
}

static
void peri_pairing_failed(void *delegate, struct net_layer_s *layer,
                         enum sm_reason reason)
{
  struct ble_peripheral_s *peri = delegate;

  printk("Slave pairing failed\n");

  peri->handler->pairing_failed(peri, reason);
}

static
void peri_pairing_success(void *delegate, struct net_layer_s *layer)
{
  struct ble_peripheral_s *peri = delegate;

  printk("Slave pairing success\n");

  peri->handler->pairing_success(peri);
}

static void peri_sm_invalidate(void *delegate, struct net_layer_s *layer)
{
  struct ble_peripheral_s *peri = delegate;

  if (peri->sm == layer)
    peri->sm = NULL;
}
#endif

static void peri_adv_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_peripheral_s *peri = delegate;

  printk("Advertise layer destroyed\n");

  peri_state_update(peri);
}

static
error_t peri_connection_create(struct ble_peripheral_s *peri,
                               const struct ble_adv_connect_s *conn,
                               dev_timer_value_t anchor)
{
  error_t err;
  struct net_layer_s *slave, *l2cap, *signalling, *gatt, *gap, *link, *llcp;
  uint16_t cid;
  struct ble_phy_params_s slave_params;

#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif

  slave_params.connect_packet_timestamp = anchor;
  slave_params.conn_req = *conn;
#if defined(CONFIG_BLE_SECURITY_DB)
  ble_peer_init(&peri->peer, &peri->context->security_db, &conn->master);
#else
  ble_peer_init(&peri->peer, NULL, &conn->master);
#endif

  err = DEVICE_OP(&peri->context->ble, layer_create,
                  &peri->context->scheduler,
                  BLE_NET_LAYER_SLAVE,
                  &slave_params,
                  peri, &peri_slave_vtable.base,
                  &slave);
  if (err) {
    printk("error while creating slave: %d\n", err);
    return err;
  }

  struct ble_link_param_s link_params = {
    .is_master = 0,
#if defined(CONFIG_BLE_CRYPTO)
    .crypto = &peri->context->crypto,
#endif
  };

  err = ble_link_create(&peri->context->scheduler, &link_params, NULL, NULL, &link);
  if (err) {
    printk("error while creating link: %d\n", err);
    goto out_slave;
  }

  err = net_layer_bind(slave, NULL, link);
  if (err) {
    printk("error while binding link to slave: %d\n", err);
    goto out_link;
  }

  err = ble_l2cap_create(&peri->context->scheduler, NULL, NULL, &l2cap);
  if (err) {
    printk("error while creating l2cap: %d\n", err);
    goto out_link;
  }

  cid = BLE_LINK_CHILD_L2CAP;
  err = net_layer_bind(link, &cid, l2cap);
  if (err) {
    printk("error while binding l2cap to link: %d\n", err);
    goto out_l2cap;
  }

  struct ble_llcp_params_s llcp_params = {
#if defined(CONFIG_BLE_CRYPTO)
    .rng = &peri->context->rng,
    .peer = &peri->peer,
#endif
  };
  err = ble_llcp_create(&peri->context->scheduler, &llcp_params, peri, &peri_llcp_vtable.base, &llcp);
  if (err) {
    printk("error while creating llcp: %d\n", err);
    goto out_l2cap;
  }

  cid = BLE_LINK_CHILD_LLCP;
  err = net_layer_bind(link, &cid, llcp);
  if (err) {
    printk("error while binding llcp to link: %d\n", err);
    goto out_llcp;
  }

#if defined(CONFIG_BLE_CRYPTO)
  struct ble_sm_param_s sm_params = {
    .peer = &peri->peer,
    .local_addr = slave_params.conn_req.slave,
    .rng = &peri->context->rng,
    .crypto = &peri->context->crypto,
  };

  err = ble_sm_create(&peri->context->scheduler, &sm_params, peri, &sm_delegate_vtable.base, &sm);
  if (err) {
    printk("error while creating sm: %d\n", err);
    goto out_llcp;
  }

  cid = BLE_L2CAP_CID_SM;
  err = net_layer_bind(l2cap, &cid, sm);
  if (err) {
    printk("error while binding sm to l2cap: %d\n", err);
    goto out_sm;
  }
#endif

  struct net_layer_s *att;

  err = ble_att_create(&peri->context->scheduler, NULL, NULL, &att);
  if (err) {
    printk("error while creating att: %d\n", err);
    goto out_sm;
  }

  cid = BLE_L2CAP_CID_ATT;
  err = net_layer_bind(l2cap, &cid, att);
  if (err) {
    printk("error while binding gatt to l2cap: %d\n", err);
    goto out_att;
  }

  struct ble_gatt_params_s gatt_params = {
    .peer = &peri->peer,
    .db = &peri->context->gattdb,
  };

  err = ble_gatt_create(&peri->context->scheduler, &gatt_params, NULL, NULL, &gatt);
  if (err) {
    printk("error while creating gatt: %d\n", err);
    goto out_att;
  }

  cid = BLE_ATT_SERVER;
  err = net_layer_bind(att, &cid, gatt);
  if (err) {
    printk("error while binding gatt to att: %d\n", err);
    goto out_gatt;
  }

  err = ble_signalling_create(&peri->context->scheduler, NULL, NULL, &signalling);
  if (err) {
    printk("error while creating signalling: %d\n", err);
    goto out_gatt;
  }

  cid = BLE_L2CAP_CID_SIGNALLING;
  err = net_layer_bind(l2cap, &cid, signalling);
  if (err) {
    printk("error while binding signalling to l2cap: %d\n", err);
    goto out_signalling;
  }

  struct ble_gap_params_s gap_params = {
    .db = &peri->context->gattdb,
    .sig = signalling,
  };
  err = ble_gap_create(&peri->context->scheduler, &gap_params, NULL, NULL, &gap);
  if (err) {
    printk("error while creating gap: %d\n", err);
    goto out_signalling;
  }

  cid = BLE_LLCP_CHILD_GAP;
  err = net_layer_bind(llcp, &cid, gap);
  if (err) {
    printk("error while binding gap to slave: %d\n", err);
    goto out_gap;
  }

  printk("Connection creation done\n");

 out_gap:
  net_layer_refdec(gap);
 out_signalling:
  net_layer_refdec(signalling);
 out_gatt:
  net_layer_refdec(gatt);
 out_att:
  net_layer_refdec(att);
 out_sm:
#if defined(CONFIG_BLE_CRYPTO)
  net_layer_refdec(sm);
#endif
 out_llcp:
  net_layer_refdec(llcp);
 out_l2cap:
  net_layer_refdec(l2cap);
 out_link:
  net_layer_refdec(link);
 out_slave:

  if (err) {
    net_layer_refdec(slave);
  } else {
#if defined(CONFIG_BLE_CRYPTO)
    peri->sm = sm;
#endif
    peri->slave = slave;
    peri->connection_tk = anchor;
  }

  return err;
}

static
bool_t peri_connection_requested(void *delegate, struct net_layer_s *layer,
                                 const struct ble_adv_connect_s *conn,
                                 dev_timer_value_t anchor)
{
  struct ble_peripheral_s *peri = delegate;
  error_t err;

  printk("Connection request from "BLE_ADDR_FMT"...", BLE_ADDR_ARG(&conn->master));

#if defined(CONFIG_BLE_CRYPTO)
  if (peri->sm) {
    printk(" still have sm\n");
    return 1;
  }
#endif

  if (peri->slave) {
    printk(" still have slave\n");
    return 1;
  }

  if (!peri->handler->connection_requested(peri, &conn->master)) {
    printk(" rejected by handler\n");
    return 1;
  }

#if defined(CONFIG_BLE_SECURITY_DB)
  if (!(peri->mode & BLE_PERIPHERAL_PAIRABLE)
      && !ble_security_db_contains(&peri->context->security_db, &conn->master)) {
    printk(" ignored: we are not paired\n");
    return 1;
  }
#endif

  err = peri_connection_create(peri, conn, anchor);

  if (!err) {
    net_layer_refdec(peri->adv);
    peri->adv = NULL;
  }

  peri_state_update(peri);

  return err ? 1 : 0;
}

static void adv_data_append(uint8_t **buffer, size_t *buffer_size,
                              const uint8_t type,
                              const void *data, const uint8_t size)
{
  if (*buffer_size < size + 2 || size == 0)
    return;

  uint8_t *ptr = *buffer;

  *ptr++ = size + 1;
  *ptr++ = type;
  memcpy(ptr, data, size);

  *buffer += size + 2;
  *buffer_size -= size + 2;
}

static error_t adv_start(struct ble_peripheral_s *peri)
{
  static const size_t srv_max_count = 4;
  static const size_t ad_size_max = 62;

  struct ble_advertiser_param_s params;
  error_t err;
  size_t value_size;
  uint16_t srv_list[srv_max_count];
  uint8_t srv16_list[16];
  uint8_t ad_[ad_size_max];
  const void *data;
  size_t size, ad_left;
  uint8_t *ad = ad_;

  if (peri->slave) {
    printk("Cannot advertise: has slave\n");
    return 0;
  }

  if (peri->adv) {
    printk("Cannot advertise: has adv\n");
    return 0;
  }

  printk("Peripheral advertising starting\n");

  params.local_addr = peri->addr;
  params.interval_ms = peri->params.adv_interval_ms;
  params.delay_max_ms = peri->params.adv_interval_ms / 16;
  if (params.delay_max_ms == 0)
    params.delay_max_ms = 1;
  params.connectable = 1;

  ad_left = ad_size_max;

  uint8_t flags = BLE_GAP_FLAGS_BREDR_NOT_SUPPORTED;
  //  if (peri->mode & BLE_PERIPHERAL_PAIRABLE)
    flags |= BLE_GAP_FLAGS_LIMITED_ADV;
  adv_data_append(&ad, &ad_left, BLE_GAP_FLAGS, &flags, sizeof(flags));

  if (!ble_gattdb_std_char_read(&peri->context->gattdb,
                                 BLE_UUID_GENERIC_ACCESS_SERVICE,
                                 BLE_UUID_GAP_APPEARANCE_CHAR,
                                 &data, &size))
    adv_data_append(&ad, &ad_left, BLE_GAP_APPEARANCE, data, size);

  value_size = ble_gattdb_srv16_list_get(&peri->context->gattdb,
                                          srv_list, sizeof(srv_list[0]) * srv_max_count);
  if (value_size)
    adv_data_append(&ad, &ad_left,
                    value_size <= srv_max_count ? BLE_GAP_UUID16_SERVICE_LIST_COMPLETE
                    : BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE,
                    srv_list, __MIN(sizeof(srv_list[0]), value_size));

  value_size = ble_gattdb_srv128_list_get(&peri->context->gattdb,
                                           srv16_list, sizeof(srv16_list));
  if (value_size)
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_UUID128_SERVICE_LIST_COMPLETE,
                    srv16_list, __MIN(sizeof(srv16_list), value_size));

  if (!ble_gattdb_std_char_read(&peri->context->gattdb,
                                 BLE_UUID_GENERIC_ACCESS_SERVICE,
                                 BLE_UUID_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_CHAR,
                                 &data, &size))
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_SLAVE_CONNECTION_INTERVAL_RANGE, data, 4);

  if (!ble_gattdb_std_char_read(&peri->context->gattdb,
                                 BLE_UUID_GENERIC_ACCESS_SERVICE,
                                 BLE_UUID_GAP_DEVICE_NAME_CHAR,
                                 &data, &size))
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_COMPLETE_LOCAL_NAME, data, size);

  params.ad = ad_;
  params.ad_len = ad - ad_;

  err = DEVICE_OP(&peri->context->ble, layer_create,
                  &peri->context->scheduler,
                  BLE_NET_LAYER_ADV,
                  &params,
                  peri, &peri_adv_vtable.base,
                  &peri->adv);
  if (err) {
    printk("Advertising start failed: %d\n", err);
    peri->adv = NULL;
  }
  
  peri_state_update(peri);

  return err;
}

error_t ble_peripheral_init(
  struct ble_peripheral_s *peri,
  const struct ble_peripheral_params_s *params,
  const struct ble_peripheral_handler_s *handler,
  struct ble_stack_context_s *context)
{
  memset(peri, 0, sizeof(*peri));

  peri->params = *params;
  peri->handler = handler;
  peri->context = context;
  peri->mode = 0;

  ble_stack_context_local_address_get(context, &peri->addr);

  return 0;
}

static const struct ble_phy_delegate_vtable_s peri_slave_vtable =
{
  .base.release = peri_slave_destroyed,
  .connection_lost = conn_connection_lost,
};

#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s sm_delegate_vtable =
{
  .base.release = peri_sm_invalidate,
  .pairing_requested = peri_pairing_requested,
  .pairing_failed = peri_pairing_failed,
  .pairing_success = peri_pairing_success,
};
#endif

static const struct ble_advertiser_delegate_vtable_s peri_adv_vtable =
{
  .base.release = peri_adv_destroyed,
  .connection_requested = peri_connection_requested,
};

static const struct ble_llcp_delegate_vtable_s peri_llcp_vtable =
{
  .base.release = peri_llcp_destroyed,
  .connection_closed = peri_llcp_closed,
};

void ble_peripheral_mode_set(struct ble_peripheral_s *peri, uint8_t mode)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  if (ble_security_db_count(&peri->context->security_db) == 0
      && mode & BLE_PERIPHERAL_CONNECTABLE) {
    mode |= BLE_PERIPHERAL_PAIRABLE;
  }
#else
  mode |= BLE_PERIPHERAL_PAIRABLE;
#endif

  if (mode == peri->mode)
    return;

  peri->mode = mode;

  if (!(mode & BLE_PERIPHERAL_CONNECTABLE))
    peri_conn_drop(peri, 0);

  if (peri->adv) {
    net_layer_refdec(peri->adv);
    peri->adv = NULL;
  }

  if (mode & BLE_PERIPHERAL_CONNECTABLE && !peri->slave)
    adv_start(peri);
}

#if defined(CONFIG_BLE_CRYPTO)
void ble_peripheral_pairing_accept(struct ble_peripheral_s *peri,
                                   bool_t mitm_protection,
                                   uint32_t pin,
                                   const void *oob_data)
{
  if (peri->sm)
    ble_sm_pairing_accept(peri->sm, mitm_protection, pin, oob_data);
}
#endif
