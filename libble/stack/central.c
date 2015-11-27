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

#include <ble/protocol/l2cap.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>

#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>

#include <ble/stack/central.h>
#include <ble/stack/context.h>

#include <ble/net/scanner.h>
#include <ble/net/phy.h>
#include <ble/net/gatt.h>
#include <ble/net/sm.h>
#include <ble/net/gap.h>
#include <ble/net/att.h>
#include <ble/net/link.h>
#include <ble/net/llcp.h>

#include <ble/net/layer_id.h>

#include <mutek/printk.h>

#include <ble/net/generic.h>

static const struct ble_phy_delegate_vtable_s ctr_master_vtable;
#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s sm_delegate_vtable;
#endif
static const struct ble_llcp_delegate_vtable_s ctr_llcp_vtable;
static const struct ble_scanner_delegate_vtable_s ctr_scan_vtable;

static void ble_central_target_add(struct ble_central_s *ctrl,
                                   const struct ble_addr_s *addr,
                                   enum ble_scanner_policy_e policy);

struct dev_rng_s;

static error_t scan_start(struct ble_central_s *ctr);

static void ctr_state_update(struct ble_central_s *ctr)
{
  enum ble_central_state_e state = BLE_CENTRAL_IDLE;

  if (ctr->master) {
    state = BLE_CENTRAL_CONNECTED;
  } else if (ctr->scan) {
    if (ctr->mode & BLE_CENTRAL_PAIRABLE)
      state = BLE_CENTRAL_PAIRING;
    else
      state = BLE_CENTRAL_SCANNING;
  }

  if (state == ctr->last_state)
    return;

  ctr->last_state = state;

  printk("Central state now %d\n", state);

  ctr->handler->state_changed(ctr, state);
}

static void ctr_conn_drop(struct ble_central_s *ctr, uint8_t reason)
{
  if (!ctr->master)
    return;

  if (ctr->llcp && !reason)
    ble_llcp_connection_close(ctr->llcp);

  net_layer_refdec(ctr->master);
  ctr->master = NULL;
  ctr->llcp = NULL;

  ctr->handler->connection_closed(ctr, reason);

  ctr_state_update(ctr);
  scan_start(ctr);
}

static void conn_connection_lost(void *delegate, struct net_layer_s *layer,
                                 uint8_t reason)
{
  struct ble_central_s *ctr = delegate;

  printk("Central connection dropped: %d\n", reason);

  if (layer != ctr->master)
    return;

  ctr_conn_drop(ctr, reason);
}

static void ctr_llcp_closed(void *delegate, struct net_layer_s *layer,
                             uint8_t reason)
{
  struct ble_central_s *ctr = delegate;

  if (layer != ctr->llcp)
    return;

  ctr->llcp = NULL;

  ctr_conn_drop(ctr, reason);
}

static void ctr_llcp_destroyed(void *delegate, struct net_layer_s *layer)
{
  printk("LLCP layer released\n");
}

static void ctr_master_destroyed(void *delegate, struct net_layer_s *layer)
{
  printk("Master layer released\n");
}

#if defined(CONFIG_BLE_CRYPTO)
static
void ctr_pairing_requested(void *delegate, struct net_layer_s *layer,
                            bool_t bonding)
{
  struct ble_central_s *ctr = delegate;

  printk("Master pairing requested\n");

  ctr->handler->pairing_requested(ctr, bonding);
}

static
void ctr_pairing_failed(void *delegate, struct net_layer_s *layer,
                         enum sm_reason reason)
{
  struct ble_central_s *ctr = delegate;

  printk("Master pairing failed\n");

  ctr->handler->pairing_failed(ctr, reason);
}

static
void ctr_pairing_success(void *delegate, struct net_layer_s *layer)
{
  struct ble_central_s *ctr = delegate;

  printk("Master pairing success\n");

  ctr->handler->pairing_success(ctr);
}

static void ctr_sm_invalidate(void *delegate, struct net_layer_s *layer)
{
  struct ble_central_s *ctr = delegate;

  if (ctr->sm == layer)
    ctr->sm = NULL;
}
#endif

static void ctr_scan_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_central_s *ctr = delegate;

  printk("Scanner layer destroyed\n");

  ctr_state_update(ctr);
}

static
error_t ctr_connection_create(struct ble_central_s *ctr,
                               const struct ble_adv_connect_s *conn,
                               dev_timer_value_t anchor)
{
  error_t err;
  struct net_layer_s *master, *l2cap, *signalling, *gatt, *gap, *link, *llcp;
  uint16_t cid;
  struct ble_phy_params_s master_params;

#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif

  master_params.connect_packet_timestamp = anchor;
  master_params.conn_req = *conn;
#if defined(CONFIG_BLE_SECURITY_DB)
  ble_peer_init(&ctr->peer, &ctr->context->security_db, &conn->master);
#else
  ble_peer_init(&ctr->peer, NULL, &conn->master);
#endif

  err = DEVICE_OP(&ctr->context->ble, layer_create,
                  &ctr->context->scheduler,
                  BLE_NET_LAYER_MASTER,
                  &master_params,
                  ctr, &ctr_master_vtable.base,
                  &master);
  if (err) {
    printk("error while creating master: %d\n", err);
    return err;
  }

  struct ble_link_param_s link_params = {
    .is_master = 0,
#if defined(CONFIG_BLE_CRYPTO)
    .crypto = &ctr->context->crypto,
#endif
  };

  err = ble_link_create(&ctr->context->scheduler, &link_params, NULL, NULL, &link);
  if (err) {
    printk("error while creating link: %d\n", err);
    goto out_master;
  }

  err = net_layer_bind(master, NULL, link);
  if (err) {
    printk("error while binding link to master: %d\n", err);
    goto out_link;
  }

  err = ble_l2cap_create(&ctr->context->scheduler, NULL, NULL, &l2cap);
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
    .rng = &ctr->context->rng,
    .peer = &ctr->peer,
#endif
  };
  err = ble_llcp_create(&ctr->context->scheduler, &llcp_params, ctr, &ctr_llcp_vtable.base, &llcp);
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
    .peer = &ctr->peer,
    .local_addr = master_params.conn_req.master,
    .rng = &ctr->context->rng,
    .crypto = &ctr->context->crypto,
  };

  err = ble_sm_create(&ctr->context->scheduler, &sm_params, ctr, &sm_delegate_vtable.base, &sm);
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

  err = ble_att_create(&ctr->context->scheduler, NULL, NULL, &att);
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
    .peer = &ctr->peer,
    .db = &ctr->context->gattdb,
  };

  err = ble_gatt_create(&ctr->context->scheduler, &gatt_params, NULL, NULL, &gatt);
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

  err = ble_signalling_create(&ctr->context->scheduler, NULL, NULL, &signalling);
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
    .db = &ctr->context->gattdb,
    .sig = signalling,
  };
  err = ble_gap_create(&ctr->context->scheduler, &gap_params, NULL, NULL, &gap);
  if (err) {
    printk("error while creating gap: %d\n", err);
    goto out_signalling;
  }

  cid = BLE_LLCP_CHILD_GAP;
  err = net_layer_bind(llcp, &cid, gap);
  if (err) {
    printk("error while binding gap to master: %d\n", err);
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
 out_master:

  if (err) {
    net_layer_refdec(master);
  } else {
#if defined(CONFIG_BLE_CRYPTO)
    ctr->sm = sm;
#endif
    ctr->master = master;
    ctr->connection_tk = anchor;
  }

  return err;
}

static
bool_t ctr_connection_requested(void *delegate, struct net_layer_s *layer,
                                 const struct ble_adv_connect_s *conn,
                                 dev_timer_value_t anchor)
{
  struct ble_central_s *ctr = delegate;
  error_t err;

  printk("Connection request from "BLE_ADDR_FMT"...", BLE_ADDR_ARG(&conn->master));

#if defined(CONFIG_BLE_CRYPTO)
  if (ctr->sm) {
    printk(" still have sm\n");
    return 1;
  }
#endif

  if (ctr->master) {
    printk(" still have master\n");
    return 1;
  }

  ctr->handler->connection_opened(ctr, &conn->master);

  err = ctr_connection_create(ctr, conn, anchor);
  if (!err) {
    net_layer_refdec(ctr->scan);
    ctr->scan = NULL;
  }

  ctr_state_update(ctr);

  return err ? 1 : 0;
}

static error_t scan_start(struct ble_central_s *ctr)
{
  error_t err;

  if (ctr->master) {
    printk("Cannot scan: has master\n");
    return 0;
  }

  if (ctr->scan) {
    printk("Cannot scan: has scan\n");
    return 0;
  }

  ctr->params.access_address = ble_stack_access_address_generate(ctr->context);
  dev_rng_wait_read(&ctr->context->rng, &ctr->params.crc_init, 4);
  ctr->params.crc_init &= 0xffffff;
  ble_stack_context_local_address_get(ctr->context, &ctr->params.local_addr);

  printk("Central scanning starting\n");

  err = DEVICE_OP(&ctr->context->ble, layer_create,
                  &ctr->context->scheduler,
                  BLE_NET_LAYER_SCANNER,
                  &ctr->params,
                  ctr, &ctr_scan_vtable.base,
                  &ctr->scan);
  if (err) {
    printk("Scanning start failed: %d\n", err);
    ctr->scan = NULL;
  }
  
  ctr_state_update(ctr);

  return err;
}

error_t ble_central_init(
  struct ble_central_s *ctr,
  const struct ble_central_params_s *params,
  const struct ble_central_handler_s *handler,
  struct ble_stack_context_s *context)
{
  memset(ctr, 0, sizeof(*ctr));

  ctr->params.interval_ms = params->scan_interval_ms;
  ctr->params.duration_ms = params->scan_duration_ms;
  ctr->params.target_count = 0;
  ctr->params.default_policy = BLE_SCANNER_IGNORE;

  ctr->handler = handler;
  ctr->context = context;
  ctr->mode = 0;

  ble_stack_context_local_address_get(context, &ctr->addr);

  return 0;
}

static const struct ble_phy_delegate_vtable_s ctr_master_vtable =
{
  .base.release = ctr_master_destroyed,
  .connection_lost = conn_connection_lost,
};

#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s sm_delegate_vtable =
{
  .base.release = ctr_sm_invalidate,
  .pairing_requested = ctr_pairing_requested,
  .pairing_failed = ctr_pairing_failed,
  .pairing_success = ctr_pairing_success,
};
#endif

static const struct ble_scanner_delegate_vtable_s ctr_scan_vtable =
{
  .base.release = ctr_scan_destroyed,
  .connection_requested = ctr_connection_requested,
};

static const struct ble_llcp_delegate_vtable_s ctr_llcp_vtable =
{
  .base.release = ctr_llcp_destroyed,
  .connection_closed = ctr_llcp_closed,
};

void ble_central_mode_set(struct ble_central_s *ctr, uint8_t mode)
{
#if defined(CONFIG_BLE_SECURITY_DB)
  if (ble_security_db_count(&ctr->context->security_db) == 0
      && mode & BLE_CENTRAL_CONNECTABLE) {
    mode |= BLE_CENTRAL_PAIRABLE;
  }
#else
  mode |= BLE_CENTRAL_PAIRABLE;
#endif

  if (mode == ctr->mode)
    return;

  ctr->mode = mode;

  if (!(mode & BLE_CENTRAL_CONNECTABLE))
    ctr_conn_drop(ctr, 0);

  if (ctr->scan) {
    net_layer_refdec(ctr->scan);
    ctr->scan = NULL;
  }

  if (mode & BLE_CENTRAL_CONNECTABLE && !ctr->master)
    scan_start(ctr);
}

#if defined(CONFIG_BLE_CRYPTO)
void ble_central_pairing_accept(struct ble_central_s *ctr,
                                   bool_t mitm_protection,
                                   uint32_t pin,
                                   const void *oob_data)
{
  if (ctr->sm)
    ble_sm_pairing_accept(ctr->sm, mitm_protection, pin, oob_data);
}
#endif

static void ble_central_target_add(struct ble_central_s *ctrl,
                                   const struct ble_addr_s *addr,
                                   enum ble_scanner_policy_e policy)
{
  if (ctrl->params.target_count < BLE_SCANNER_TARGET_MAXCOUNT) {
    ctrl->params.target_count++;
  } else {
    memmove(ctrl->params.target, ctrl->params.target + 1,
            sizeof(struct ble_scanner_target_s) * (BLE_SCANNER_TARGET_MAXCOUNT - 1));
  }

  ctrl->params.target[ctrl->params.target_count - 1].addr = *addr;
  ctrl->params.target[ctrl->params.target_count - 1].policy = policy;

  if (ctrl->scan)
    ble_scanner_params_update(ctrl->scan, &ctrl->params);
}

void ble_central_connect(struct ble_central_s *ctrl, const struct ble_addr_s *addr)
{
  ble_central_target_add(ctrl, addr, BLE_SCANNER_CONNECT);
}

