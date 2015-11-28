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
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>
#include <ble/protocol/l2cap.h>

#include <ble/net/phy.h>
#include <ble/net/gatt.h>
#include <ble/net/att.h>
#include <ble/net/link.h>
#include <ble/net/llcp.h>
#include <ble/net/sm.h>
#include <ble/net/gap.h>
#include <ble/net/layer_id.h>
#include <ble/net/generic.h>

#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>

#include <mutek/printk.h>

static SLAB_GROW(packet_pool_grow)
{
    return 40;
}

error_t ble_stack_context_init(struct ble_stack_context_s *ctx,
                               const char *ble_name,
                               const char *rtc_name,
                               const char *rng_name,
                               const char *sec_name,
                               const char *persist_name)
{
  error_t err;
  struct dev_rng_s rng;

  err = dev_rng_init(&ctx->rng, sec_name);
  if (err) {
    printk("Error while initing DRBG: %d\n", err);
    return err;
  }

  err = dev_rng_init(&rng, rng_name);
  if (err) {
    printk("Error while initing RNG: %d\n", err);
    goto rng_close;
  }

  err = dev_rng_wait_seed_from_other(&ctx->rng, &rng, 16);
  if (err) {
    printk("Error while seeding from RNG: %d\n", err);
    goto rng_close2;
  }

  dev_rng_cleanup(&rng);

#if defined(CONFIG_BLE_CRYPTO)
  err = device_get_accessor_by_path(&ctx->crypto, NULL, sec_name, DRIVER_CLASS_CRYPTO);
  if (err) {
    printk("Error while opening crypto device: %d\n", err);
    goto rng_close;
  }
#endif

  err = device_get_accessor_by_path(&ctx->ble, NULL, ble_name, DRIVER_CLASS_NET);
  if (err) {
    printk("Error while opening BLE network device: %d\n", err);
    goto crypto_close;
  }

  buffer_pool_init(&ctx->packet_pool, CONFIG_BLE_PACKET_SIZE,
                   packet_pool_grow, mem_scope_sys);

  ble_gattdb_init(&ctx->gattdb);

  err = net_scheduler_init(&ctx->scheduler, &ctx->packet_pool, rtc_name);
  if (err) {
    printk("Error while initializing net scheduler: %d\n", err);
    goto gatt_db_cleanup;
  }

#if defined(CONFIG_BLE_CRYPTO)
  err = ble_security_db_init(&ctx->security_db, persist_name, sec_name, &ctx->rng);
  if (err) {
    printk("Error while initializing peer db: %d\n", err);
    goto sched_cleanup;
  }
#endif

  return 0;

 sched_cleanup:
  net_scheduler_cleanup(&ctx->scheduler);
 gatt_db_cleanup:
  ble_gattdb_cleanup(&ctx->gattdb);
  buffer_pool_cleanup(&ctx->packet_pool);
 crypto_close:
#if defined(CONFIG_BLE_CRYPTO)
  device_put_accessor(&ctx->crypto);
  goto rng_close;
#endif
 rng_close2:
  dev_rng_cleanup(&rng);
 rng_close:
  dev_rng_cleanup(&ctx->rng);

  return err;
}

void ble_stack_context_cleanup(struct ble_stack_context_s *ctx)
{
  net_scheduler_cleanup(&ctx->scheduler);
  ble_gattdb_cleanup(&ctx->gattdb);
  buffer_pool_cleanup(&ctx->packet_pool);
#if defined(CONFIG_BLE_CRYPTO)
  ble_security_db_cleanup(&ctx->security_db);
#endif
  dev_rng_cleanup(&ctx->rng);
}

error_t ble_stack_context_address_non_resolvable_generate(struct ble_stack_context_s *ctx,
                                                          struct ble_addr_s *addr)
{
  error_t err;

  err = dev_rng_wait_read(&ctx->rng, addr->addr, 6);
  if (err)
    return err;

  ble_addr_random_type_set(addr, BLE_ADDR_RANDOM_NON_RESOLVABLE);

  return 0;
}

error_t ble_stack_context_local_address_get(struct ble_stack_context_s *ctx,
                                            struct ble_addr_s *addr)
{
  error_t err;
  struct dev_net_info_s info;

  err = DEVICE_OP(&ctx->ble, get_info, &info);
  if (err)
    return err;

  ble_addr_net_parse(addr, &info.addr);

  return 0;
}

uint32_t ble_stack_access_address_generate(struct ble_stack_context_s *ctx)
{
  for (;;) {
    uint8_t tmp[16];
  
    dev_rng_wait_read(&ctx->rng, tmp, 16);

    for (uint8_t offset = 0; offset <= 12; ++offset) {
      uint32_t aa = endian_le32_na_load(tmp + offset);

      if (ble_data_aa_is_valid(aa))
        return aa;
    }
  }
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

void ble_stack_context_ad_collect(struct ble_stack_context_s *context,
                            uint8_t *ad_, size_t ad_size_max,
                            size_t *ad_size_used)
{
  static const size_t srv_max_count = 4;
  uint16_t srv16_list[srv_max_count];
  uint8_t srv128_list[16];
  const void *data;
  size_t size, ad_left, value_size;
  uint8_t *ad = ad_;

  ad_left = ad_size_max;

  uint8_t flags = BLE_GAP_FLAGS_BREDR_NOT_SUPPORTED;
  flags |= BLE_GAP_FLAGS_LIMITED_ADV;
  adv_data_append(&ad, &ad_left, BLE_GAP_FLAGS, &flags, sizeof(flags));

  if (!ble_gattdb_std_char_read(&context->gattdb,
                                 BLE_UUID_GENERIC_ACCESS_SERVICE,
                                 BLE_UUID_GAP_APPEARANCE_CHAR,
                                 &data, &size))
    adv_data_append(&ad, &ad_left, BLE_GAP_APPEARANCE, data, size);

  value_size = ble_gattdb_srv16_list_get(&context->gattdb, srv16_list, sizeof(srv16_list));
  if (value_size)
    adv_data_append(&ad, &ad_left,
                    value_size <= srv_max_count ? BLE_GAP_UUID16_SERVICE_LIST_COMPLETE
                    : BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE,
                    srv16_list, __MIN(sizeof(srv16_list), value_size));

  value_size = ble_gattdb_srv128_list_get(&context->gattdb, srv128_list, sizeof(srv128_list));
  if (value_size)
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_UUID128_SERVICE_LIST_COMPLETE,
                    srv128_list, __MIN(sizeof(srv128_list), value_size));

  if (!ble_gattdb_std_char_read(&context->gattdb,
                                 BLE_UUID_GENERIC_ACCESS_SERVICE,
                                 BLE_UUID_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_CHAR,
                                 &data, &size))
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_SLAVE_CONNECTION_INTERVAL_RANGE, data, 4);

  if (!ble_gattdb_std_char_read(&context->gattdb,
                                 BLE_UUID_GENERIC_ACCESS_SERVICE,
                                 BLE_UUID_GAP_DEVICE_NAME_CHAR,
                                 &data, &size))
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_COMPLETE_LOCAL_NAME, data, size);

  *ad_size_used = ad_size_max - ad_left;
}

static const struct ble_phy_delegate_vtable_s conn_phy_vtable;
#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s conn_sm_vtable;
#endif
static const struct ble_llcp_delegate_vtable_s conn_llcp_vtable;

static void conn_connection_lost(void *delegate, struct net_layer_s *layer,
                                 uint8_t reason)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Connection dropped: %d\n", reason);

  if (layer != conn->phy)
    return;

  ble_stack_connection_drop(conn, reason);
}

static void conn_llcp_closed(void *delegate, struct net_layer_s *layer,
                             uint8_t reason)
{
  struct ble_stack_connection_s *conn = delegate;

  if (layer != conn->llcp)
    return;

  conn->llcp = NULL;

  ble_stack_connection_drop(conn, reason);
}

static void conn_llcp_destroyed(void *delegate, struct net_layer_s *layer)
{
  printk("LLCP layer released\n");
}

static void conn_slave_destroyed(void *delegate, struct net_layer_s *layer)
{
  printk("Slave layer released\n");
}

#if defined(CONFIG_BLE_CRYPTO)
static
void conn_pairing_requested(void *delegate, struct net_layer_s *layer,
                            bool_t bonding)
{
  struct ble_stack_connection_s *conn = delegate;

  printk("Slave pairing requested\n");

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

  printk("Slave pairing success\n");

  conn->chandler->pairing_success(conn);
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
                                            dev_timer_value_t anchor)
{
  error_t err;
  struct net_layer_s *phy, *l2cap, *signalling, *gatt, *gap, *link, *llcp;
  uint16_t cid;
  struct ble_phy_params_s phy_params;

#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif

  phy_params.connect_packet_timestamp = anchor;
  phy_params.conn_req = *conn_params;
#if defined(CONFIG_BLE_SECURITY_DB)
  ble_peer_init(&conn->peer, &context->security_db, &conn_params->master);
#else
  ble_peer_init(&conn->peer, NULL, &conn_params->master);
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
    .is_master = is_master,
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
  };
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
    .local_addr = phy_params.conn_req.slave,
    .rng = &context->rng,
    .crypto = &context->crypto,
  };

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

  struct net_layer_s *att;

  err = ble_att_create(&context->scheduler, NULL, NULL, &att);
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
    conn->connection_tk = anchor;
    conn->chandler = chandler;
    conn->handler = handler;
  }

  return err;
}

void ble_stack_connection_drop(struct ble_stack_connection_s *conn,
                                 uint8_t reason)
{
  if (!conn->phy)
    return;

  if (conn->llcp && !reason)
    ble_llcp_connection_close(conn->llcp);

  net_layer_refdec(conn->phy);
  conn->phy = NULL;
  conn->llcp = NULL;

  conn->chandler->connection_closed(conn, reason);
}

static const struct ble_phy_delegate_vtable_s conn_slave_vtable =
{
  .base.release = conn_slave_destroyed,
  .connection_lost = conn_connection_lost,
};

#if defined(CONFIG_BLE_CRYPTO)
static const struct ble_sm_delegate_vtable_s conn_sm_vtable =
{
  .base.release = conn_sm_invalidate,
  .pairing_requested = conn_pairing_requested,
  .pairing_failed = conn_pairing_failed,
  .pairing_success = conn_pairing_success,
};
#endif

static const struct ble_llcp_delegate_vtable_s conn_llcp_vtable =
{
  .base.release = conn_llcp_destroyed,
  .connection_closed = conn_llcp_closed,
};

#if defined(CONFIG_BLE_CRYPTO)
void ble_stack_connection_pairing_accept(struct ble_stack_connection_s *conn,
                                         bool_t mitm_protection,
                                         uint32_t pin,
                                         const void *oob_data)
{
  if (conn->sm)
    ble_sm_pairing_accept(conn->sm, mitm_protection, pin, oob_data);
}

void ble_stack_connection_pairing_request(struct ble_stack_connection_s *conn,
                                         bool_t mitm_protection,
                                         bool_t bonding)
{
  if (conn->sm)
    ble_sm_pairing_request(conn->sm, mitm_protection, bonding);
}

void ble_stack_connection_pairing_abort(struct ble_stack_connection_s *conn,
                                       uint8_t reason)
{
  if (conn->sm)
    ble_sm_pairing_abort(conn->sm, reason);
}
#endif
