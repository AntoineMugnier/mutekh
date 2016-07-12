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

#include <ble/stack/context.h>
#include <ble/stack/connection.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>
#include <ble/protocol/l2cap.h>

#include <ble/crypto.h>

#include <ble/net/phy.h>
#include <ble/net/gatt.h>
#include <ble/net/att.h>
#include <ble/net/link.h>
#include <ble/net/llcp.h>
#include <ble/net/sm.h>
#include <ble/net/gap.h>
#include <ble/net/layer_id.h>
#include <ble/net/generic.h>

#include <mutek/printk.h>

static const struct ble_phy_delegate_vtable_s conn_phy_vtable;
#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s conn_sm_vtable;
#endif
static const struct ble_llcp_delegate_vtable_s conn_llcp_vtable;
static const struct net_layer_delegate_vtable_s conn_att_vtable;

static void conn_state_update(struct ble_stack_connection_s *conn)
{
  conn->handler->state_changed(conn, conn->llcp || conn->phy);
}

static void ble_stack_connection_dropped(struct ble_stack_connection_s *conn,
                                          uint8_t reason)
{
  if (!conn->phy || !conn->llcp)
    return;

  printk("Connection dropped: %d\n", reason);

  net_layer_refdec(conn->phy);
  conn->phy = NULL;
#if defined(CONFIG_BLE_CRYPTO)
  conn->sm = NULL;
#endif
  conn->llcp = NULL;

  conn->chandler->connection_closed(conn, reason);
  conn_state_update(conn);
}

static void conn_phy_lost(void *delegate, struct net_layer_s *layer, uint8_t reason)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Connection PHY dropped: %d\n", reason);

  if (layer != conn->phy)
    return;

  ble_stack_connection_dropped(conn, reason);
}

static void conn_llcp_closed(void *delegate, struct net_layer_s *layer, uint8_t reason)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Connection LLCP dropped: %d\n", reason);

  if (layer != conn->llcp)
    return;

  ble_stack_connection_dropped(conn, reason);
}

static void conn_llcp_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("LLCP layer released\n");
  if (layer != conn->llcp)
    return;

  conn->llcp = NULL;
  conn_state_update(conn);
}

static void conn_phy_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("PHY layer released\n");
  if (layer != conn->phy)
    return;

  conn->phy = NULL;
  conn_state_update(conn);
}

static void conn_att_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("ATT layer released\n");
  if (layer != conn->att)
    return;

  conn->att = NULL;
  conn_state_update(conn);
}

#if defined(CONFIG_BLE_CRYPTO)
static
void conn_pairing_requested(void *delegate, struct net_layer_s *layer,
                            bool_t bonding)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Pairing requested\n");

  conn->chandler->pairing_requested(conn, bonding);
}

static
void conn_pairing_failed(void *delegate, struct net_layer_s *layer,
                         enum sm_reason reason)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Slave pairing failed\n");

  conn->chandler->pairing_failed(conn, reason);
}

static
void conn_pairing_success(void *delegate, struct net_layer_s *layer)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Conn pairing success\n");

  conn->handler->pairing_success(conn);
}

static
void conn_bonding_success(void *delegate, struct net_layer_s *layer)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Conn bonding success\n");

  conn->chandler->bonding_success(conn);
}

static void conn_sm_invalidate(void *delegate, struct net_layer_s *layer)
{
  struct ble_stack_connection_s *conn = delegate;

  if (conn->sm == layer)
    conn->sm = NULL;
}
#endif

error_t ble_stack_connection_create(struct ble_stack_connection_s *conn,
                                    struct ble_stack_context_s *context,
                                    const struct ble_stack_connection_handler_s *chandler,
                                    const struct ble_stack_context_handler_s *handler,
                                    bool_t is_master,
                                    const struct ble_adv_connect_s *conn_params,
                                    const struct ble_gap_preferred_conn_params_s *wanted_timing,
                                    dev_timer_value_t anchor)
{
  error_t err;
  struct net_layer_s *phy, *l2cap, *signalling, *att, *gatt, *gap, *link, *llcp;
  uint16_t cid;
  struct ble_phy_params_s phy_params;

#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif

  conn->is_master = is_master;
  phy_params.connect_packet_timestamp = anchor;
  phy_params.conn_req = *conn_params;
#if defined(CONFIG_BLE_CRYPTO)
  ble_peer_init(&conn->peer, &context->security_db,
                is_master ? &conn_params->slave : &conn_params->master);
#else
  ble_peer_init(&conn->peer, NULL,
                is_master ? &conn_params->slave : &conn_params->master);
#endif

  err = DEVICE_OP(&context->ble, layer_create,
                  &context->scheduler,
                  is_master ? BLE_NET_LAYER_MASTER : BLE_NET_LAYER_SLAVE,
                  &phy_params,
                  conn, &conn_phy_vtable.base,
                  &phy);
  if (err) {
    printk("error while creating phy: %d\n", err);
    return err;
  }

  struct ble_link_param_s link_params = {
#if defined(CONFIG_BLE_CRYPTO)
    .crypto = &context->crypto,
#endif
  };

  err = ble_link_create(&context->scheduler, &link_params, NULL, NULL, &link);
  if (err) {
    printk("error while creating link: %d\n", err);
    goto out_phy;
  }

  err = net_layer_bind(phy, NULL, link);
  if (err) {
    printk("error while binding link to phy: %d\n", err);
    goto out_link;
  }

  err = ble_l2cap_create(&context->scheduler, NULL, NULL, &l2cap);
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
    .rng = &context->rng,
    .peer = &conn->peer,
#endif
    .conn_timing = conn_params->timing,
  };

  if (wanted_timing) {
    llcp_params.wanted_timing = *wanted_timing;
  } else {
    llcp_params.wanted_timing.interval_min = conn_params->timing.interval;
    llcp_params.wanted_timing.interval_max = conn_params->timing.interval;
    llcp_params.wanted_timing.latency = conn_params->timing.latency;
    llcp_params.wanted_timing.timeout = conn_params->timing.timeout;
  }

  err = ble_llcp_create(&context->scheduler, &llcp_params, conn, &conn_llcp_vtable.base, &llcp);
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
    .peer = &conn->peer,
    .rng = &context->rng,
    .crypto = &context->crypto,
  };

  if (is_master)
    sm_params.local_addr = conn_params->master;
  else
    sm_params.local_addr = conn_params->slave;
    
  err = ble_sm_create(&context->scheduler, &sm_params, conn, &conn_sm_vtable.base, &sm);
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

  err = ble_att_create(&context->scheduler, conn, &conn_att_vtable, &att);
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
    .peer = &conn->peer,
    .db = &context->gattdb,
  };

  err = ble_gatt_create(&context->scheduler, &gatt_params, NULL, NULL, &gatt);
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

  err = ble_signalling_create(&context->scheduler, NULL, NULL, &signalling);
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
    .db = &context->gattdb,
    .sig = signalling,
  };
  err = ble_gap_create(&context->scheduler, &gap_params, NULL, NULL, &gap);
  if (err) {
    printk("error while creating gap: %d\n", err);
    goto out_signalling;
  }

  cid = BLE_LLCP_CHILD_GAP;
  err = net_layer_bind(llcp, &cid, gap);
  if (err) {
    printk("error while binding gap to phy: %d\n", err);
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
 out_phy:

  if (err) {
    net_layer_refdec(phy);
  } else {
#if defined(CONFIG_BLE_CRYPTO)
    conn->sm = sm;
#endif
    conn->phy = phy;
    conn->llcp = llcp;
    conn->att = att;
    conn->connection_tk = anchor;
    conn->chandler = chandler;
    conn->handler = handler;
    conn_state_update(conn);
  }

  return err;
}

void ble_stack_connection_drop(struct ble_stack_connection_s *conn)
{
  if (!conn->phy || !conn->llcp)
    return;

  ble_llcp_connection_close(conn->llcp);
}

static const struct ble_phy_delegate_vtable_s conn_phy_vtable =
{
  .base.release = conn_phy_destroyed,
  .connection_lost = conn_phy_lost,
};

#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s conn_sm_vtable =
{
  .base.release = conn_sm_invalidate,
  .pairing_requested = conn_pairing_requested,
  .pairing_failed = conn_pairing_failed,
  .pairing_success = conn_pairing_success,
  .bonding_success = conn_bonding_success,
};
#endif

static const struct ble_llcp_delegate_vtable_s conn_llcp_vtable =
{
  .base.release = conn_llcp_destroyed,
  .connection_closed = conn_llcp_closed,
};

static const struct net_layer_delegate_vtable_s conn_att_vtable =
{
  .release = conn_att_destroyed,
};

#if defined(CONFIG_BLE_CRYPTO)
void ble_stack_connection_pairing_accept(struct ble_stack_connection_s *conn,
                                         bool_t mitm_protection,
                                         uint32_t pin,
                                         const void *oob_data)
{
  printk("Pairing accepted\n");

  if (conn->sm)
    ble_sm_pairing_accept(conn->sm, mitm_protection, pin, oob_data);
}

void ble_stack_connection_pairing_request(struct ble_stack_connection_s *conn,
                                         bool_t mitm_protection,
                                         bool_t bonding)
{
  printk("Pairing requesting\n");

  if (conn->sm)
    ble_sm_pairing_request(conn->sm, mitm_protection, bonding);
}

void ble_stack_connection_pairing_abort(struct ble_stack_connection_s *conn,
                                       uint8_t reason)
{
  printk("Pairing abort\n");

  if (conn->sm)
    ble_sm_pairing_abort(conn->sm, reason);
}
#endif