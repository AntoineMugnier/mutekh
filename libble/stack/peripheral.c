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

#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>

#include <ble/net/adv.h>
#include <ble/net/phy.h>
#include <ble/net/gatt.h>
#include <ble/net/att.h>
#include <ble/net/link.h>
#include <ble/net/llcp.h>

#include <ble/net/layer_id.h>

#include <mutek/printk.h>

#include <ble/net/generic.h>

#include <ble/stack/context.h>
#include <ble/stack/connection.h>
#include <ble/stack/peripheral.h>

static const struct ble_advertiser_delegate_vtable_s peri_adv_vtable;

static error_t adv_start(struct ble_peripheral_s *peri);

struct dev_rng_s;

static void peri_state_update(struct ble_peripheral_s *peri)
{
  enum ble_peripheral_state_e state = BLE_PERIPHERAL_IDLE;

  if (peri->conn.phy) {
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

static void peri_adv_destroyed(void *delegate, struct net_layer_s *layer)
{
  struct ble_peripheral_s *peri = delegate;

  printk("Advertise layer destroyed\n");

  peri_state_update(peri);
}

static void peri_conn_state_changed(struct ble_stack_connection_s *conn,
                                    bool_t connected)
{
  struct ble_peripheral_s *peri = ble_peripheral_s_from_conn(conn);

  printk("Conn state changed %d\n", connected);

  peri_state_update(peri);

  if (!connected && peri->mode & BLE_PERIPHERAL_CONNECTABLE)
    adv_start(peri);
}

static
void peri_conn_pairing_success(struct ble_stack_connection_s *conn)
{
  struct ble_peripheral_s *peri = ble_peripheral_s_from_conn(conn);

  (void)peri;

  printk("Slave pairing success\n");
}

static const struct ble_stack_context_handler_s peri_conn_handler =
{
  .state_changed = peri_conn_state_changed,
  .pairing_success = peri_conn_pairing_success,
};

static
bool_t peri_connection_requested(void *delegate, struct net_layer_s *layer,
                                 const struct ble_adv_connect_s *conn,
                                 dev_timer_value_t anchor)
{
  struct ble_peripheral_s *peri = delegate;
  error_t err;
  const struct ble_gap_preferred_conn_params_s *wanted_timing;
  size_t size;

  printk("Connection request from "BLE_ADDR_FMT"...", BLE_ADDR_ARG(&conn->master));

#if defined(CONFIG_BLE_CRYPTO)
  if (peri->conn.sm) {
    printk(" still have sm\n");
    return 1;
  }
#endif

  if (peri->conn.phy) {
    printk(" still have phy\n");
    return 1;
  }

  if (!peri->handler->connection_requested(peri, &conn->master)) {
    printk(" rejected by handler\n");
    return 1;
  }

#if defined(CONFIG_BLE_SECURITY_DB)
  bool_t known_device = ble_security_db_contains(&peri->context->security_db, &conn->master);

  if (!(peri->mode & BLE_PERIPHERAL_PAIRABLE) && !known_device) {
    printk(" ignored: we are not paired\n");
    return 1;
  }

  /* if ((peri->mode & BLE_PERIPHERAL_PAIRABLE) && known_device) { */
  /*   printk(" ignored: we are paired and connection comes from a known device\n"); */
  /*   return 1; */
  /* } */
#endif

  if (ble_gattdb_std_char_read(&peri->context->gattdb,
                               BLE_GATT_SERVICE_GENERIC_ACCESS,
                               BLE_GATT_CHAR_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
                               (const void**)&wanted_timing, &size))
    wanted_timing = NULL;

  if (wanted_timing) {
    printk("Wanted timing: %d-%d %d %d\n",
           wanted_timing->interval_min, wanted_timing->interval_max,
           wanted_timing->latency, wanted_timing->timeout);
  } else {
    printk("Wanted timing: unknown\n");
  }

  err = ble_stack_connection_create(&peri->conn, peri->context,
                                    &peri->handler->base,
                                    &peri_conn_handler,
                                    0, conn, wanted_timing, anchor);

  if (!err) {
    net_layer_refdec(peri->adv);
    peri->adv = NULL;
  }

  peri_state_update(peri);

  return err ? 1 : 0;
}

static error_t adv_start(struct ble_peripheral_s *peri)
{
  struct ble_advertiser_param_s params;
  uint8_t ad[62];
  error_t err;

  if (peri->conn.phy) {
    printk("Cannot advertise: has phy\n");
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
  params.ad = ad;
  ble_stack_context_ad_collect(peri->context, ad, sizeof(ad), &params.ad_len);

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

static const struct ble_advertiser_delegate_vtable_s peri_adv_vtable =
{
  .base.release = peri_adv_destroyed,
  .connection_requested = peri_connection_requested,
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
    ble_stack_connection_drop(&peri->conn);

  if (peri->adv) {
    net_layer_refdec(peri->adv);
    peri->adv = NULL;
  }

  if (mode & BLE_PERIPHERAL_CONNECTABLE && !peri->conn.phy)
    adv_start(peri);
}
