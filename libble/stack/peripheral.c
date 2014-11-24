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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <ble/stack/peripheral.h>
#include <ble/protocol/l2cap.h>

#include <mutek/printk.h>

struct dev_rng_s;

static void slave_destroyed(struct ble_slave_s *slave)
{
  struct ble_peripheral_s *peri = ble_peripheral_s_from_slave(slave);

  peri->handler->dropped(peri, slave->reason);
}

static const struct ble_slave_handler_s slave_handler =
{
  .destroyed = slave_destroyed,
};

static const struct ble_l2cap_handler_s l2cap_handler =
{
  .destroyed = (void*)memory_allocator_push,
};

static const struct ble_sm_handler_s sm_handler =
{
  .destroyed = (void*)memory_allocator_push,
};

static const struct ble_signalling_handler_s sig_handler =
{
  .destroyed = (void*)memory_allocator_push,
};

static const struct ble_gatt_handler_s gatt_handler =
{
  .destroyed = (void*)memory_allocator_push,
};

static const struct ble_gap_handler_s gap_handler =
{
  .destroyed = (void*)memory_allocator_push,
};

error_t ble_peripheral_init(
  struct ble_peripheral_s *peri,
  struct net_scheduler_s *sched,
  const struct ble_peripheral_handler_s *handler,
  const char *radio,
  struct dev_rng_s *rng,
  const char *aes_dev,
  struct ble_gatt_db_s *gattdb,
  struct ble_peer_s *peer,
  dev_timer_value_t connection_packet_time,
  const struct buffer_s *connect_packet)
{
  error_t err;
  struct ble_connection_parameters_s params;
  struct ble_l2cap_s *l2cap;
  struct ble_sm_s *sm;
  struct ble_signalling_s *signalling;
  struct ble_gatt_s *gatt;
  struct ble_gap_s *gap;

  peri->handler = handler;

  err = ble_adv_connect_parse(connect_packet, &params.conn_req);
  if (err)
    return err;

  params.connect_packet_timestamp = connection_packet_time;

  err = ble_slave_init(&peri->slave, &slave_handler,
                       sched, radio, aes_dev, rng, &params, peer);
  if (err) {
    printk("error while initing slave: %d\n", err);
    return err;
  }

  l2cap = mem_alloc(sizeof(*l2cap), mem_scope_sys);
  if (!l2cap) {
    err = -ENOMEM;
    goto out_slave;
  }
  err = ble_l2cap_init(l2cap, &l2cap_handler, sched);
  if (err) {
    printk("error while initing l2cap: %d\n", err);
    mem_free(l2cap);
    goto out_slave;
  }

  sm = mem_alloc(sizeof(*sm), mem_scope_sys);
  if (!sm) {
   err = -ENOMEM;
   goto out_l2cap;
  }

  err = ble_sm_init(sm, &sm_handler, sched, peer, &params.conn_req.slave, rng, aes_dev);
  if (err) {
    mem_free(sm);
    goto out_l2cap;
  }

  gatt = mem_alloc(sizeof(*gatt), mem_scope_sys);
  if (!gatt) {
    err = -ENOMEM;
    goto out_sm;
  }
  err = ble_gatt_init(gatt, &gatt_handler, sched, gattdb);
  if (err) {
    printk("error while initing gatt: %d\n", err);
    mem_free(gatt);
    goto out_sm;
  }

  signalling = mem_alloc(sizeof(*signalling), mem_scope_sys);
  if (!signalling) {
    err = -ENOMEM;
    goto out_gatt;
  }
  err = ble_signalling_init(signalling, &sig_handler, sched);
  if (err) {
    printk("error while initing signalling: %d\n", err);
    mem_free(signalling);
    goto out_gatt;
  }

  gap = mem_alloc(sizeof(*gap), mem_scope_sys);
  if (!gap) {
    err = -ENOMEM;
    goto out_signalling;
  }
  err = ble_gap_init(gap, &gap_handler, sched, &signalling->layer);
  if (err) {
    printk("error while initing gap: %d\n", err);
    mem_free(gap);
    goto out_signalling;
  }

  /* After this point, any error goes to out: and is handled through
     refdecs.
  */

  uint16_t cid = BLE_L2CAP_CID_SM;
  err = net_layer_bind(&l2cap->layer, &cid, &sm->layer);
  if (err)
    goto out;

  cid = BLE_L2CAP_CID_ATT;
  err = net_layer_bind(&l2cap->layer, &cid, &gatt->layer);
  if (err) {
    printk("error while binding gatt to l2cap: %d\n", err);
    goto out;
  }

  cid = BLE_L2CAP_CID_SIGNALLING;
  err = net_layer_bind(&l2cap->layer, &cid, &signalling->layer);
  if (err) {
    printk("error while binding signalling to l2cap: %d\n", err);
    goto out;
  }

  // Layer bindings to slave
  err = net_layer_bind(&peri->slave.layer, NULL, &gap->layer);
  if (err) {
    printk("error while binding gap to slave: %d\n", err);
    goto out;
  }

  err = net_layer_bind(&peri->slave.layer, NULL, &l2cap->layer);
  if (err) {
    printk("error while binding l2cap to slave: %d\n", err);
    goto out;
  }

 out:
  net_layer_refdec(&gap->layer);
 out_signalling:
  net_layer_refdec(&signalling->layer);
 out_gatt:
  net_layer_refdec(&gatt->layer);
 out_sm:
  net_layer_refdec(&sm->layer);
 out_l2cap:
  net_layer_refdec(&l2cap->layer);
 out_slave:
  net_layer_refdec(&peri->slave.layer);

  return err;
}
