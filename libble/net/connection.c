#include <mutek/printk.h>
#include <mutek/buffer.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/connection.h>

#define F_SLAVE 1
#define F_REQ_SCHEDULED 2
#define F_TASK_SCHEDULED 4

static const struct net_layer_handler_s connection_handler;
static KROUTINE_EXEC(connection_rq_done);
static void ble_connection_closed(struct ble_connection_s *conn, uint8_t reason);

static void request_schedule(struct ble_connection_s *conn)
{
  dev_timer_delay_t to_event, ws;
  uint8_t channel;

  if (conn->flags & F_SLAVE) {
    if (conn->flags & F_REQ_SCHEDULED) {
      if (conn->next_event_counter == conn->last_event_counter + conn->event_progression)
        return;

      DEVICE_OP(&conn->radio, f_cancel, &conn->request);
      conn->flags &= ~F_REQ_SCHEDULED;
    }

    dev_timer_delay_t terr = 0, to_event_us;
    conn->next_event_counter = conn->last_event_counter + conn->event_progression;
    to_event_us = conn->event_progression * conn->interval_us;

    terr = to_event_us * conn->sca_ppm / 1000000;

    to_event_us -= terr;
    dev_timer_init_sec_round(&conn->timer, &to_event, NULL, to_event_us, 1000000);
    dev_timer_init_sec_round(&conn->timer, &ws, NULL, conn->window_us + 2 * terr, 1000000);

    channel = (conn->unmapped_channel + conn->event_progression * conn->hop) % 37;
  } else {
    if (conn->flags & F_REQ_SCHEDULED)
      return;

    conn->next_event_counter = conn->last_event_counter + 1;
    to_event = conn->interval_ticks;
    ws = conn->window_ticks;

    channel = conn->unmapped_channel + 1;
    if (channel >= 37)
      channel -= 37;
  }

  conn->request.data.channel = conn->mapped_channel[channel];
  conn->request.data.white_iv = channel;

  conn->request.not_before = conn->last_anchor + to_event;
  conn->request.not_after = conn->last_anchor + to_event + ws;
  conn->request.max_duration = 0;

  if (conn->request.not_before > conn->drops_at) {
    ble_connection_closed(conn, BLE_CONNECTION_TIMEOUT);
    return;
  }

  kroutine_init(&conn->request.base.kr, connection_rq_done, KROUTINE_INTERRUPTIBLE);

  DEVICE_OP(&conn->radio, f_request, &conn->request);
  conn->flags |= F_REQ_SCHEDULED;
}

static void timing_params_apply(
  struct ble_connection_s *conn,
  struct ble_conn_timing_param_s *timing)
{
  conn->interval_us = BLE_T_CONN_UNIT * timing->interval;
  conn->window_us = BLE_T_CONN_UNIT * timing->win_size;

  dev_timer_init_sec_round(&conn->timer, &conn->window_ticks,
                           NULL, conn->window_us, 1000000);

  dev_timer_init_sec_round(&conn->timer, &conn->interval_ticks,
                           NULL, conn->interval_us, 1000000);

  dev_timer_init_sec_round(&conn->timer, &conn->timeout_ticks,
                           NULL, timing->timeout, 100);

  conn->slave_latency = timing->latency;
  conn->event_progression = 1;
}

static void event_reference_setup(
  struct ble_connection_s *conn,
  dev_timer_value_t reference,
  uint16_t win_offset)
{
  dev_timer_delay_t connect_to_first, connect_to_first_ticks, terr = 0;

  connect_to_first
    = BLE_PACKET_TIME(34) + BLE_T_CONN_UNIT
    + BLE_T_CONN_UNIT * win_offset;

  dev_timer_init_sec_round(&conn->timer, &connect_to_first_ticks,
                           NULL, connect_to_first + terr, 1000000);

  conn->last_anchor = reference + connect_to_first - conn->interval_ticks;
  conn->last_event_counter = -1;
  conn->unmapped_channel = 0;
}

error_t ble_connection_init(
  struct ble_connection_s *conn,
  const struct ble_connection_handler_s *handler,
  struct net_scheduler_s *scheduler,
  const char *dev,
  const struct ble_connection_parameters_s *conn_params)
{
  error_t err;

  err = device_get_accessor_by_path(&conn->radio, NULL, dev, DRIVER_CLASS_BLE_RADIO);
  if (err)
    goto err_out;

  err = device_get_accessor_by_path(&conn->timer, NULL, dev, DRIVER_CLASS_BLE_TIMER);
  if (err)
    goto err_put_radio;

  err = net_layer_init(&conn->layer, &connection_handler, scheduler, BLE_LAYER_TYPE_L2CAP);
  if (err)
    goto err_put_devices;

  conn->flags = conn_params->is_master ? 0 : F_SLAVE;

  device_start(&conn->radio);

  buffer_queue_init(conn->reliable_queue);
  buffer_queue_init(conn->bulk_queue);

  ble_connection_channel_mapping_expand(conn->mapped_channel, p->conn_req.channel_map);
  timing_params_apply(conn, &conn_params->conn_req.timing);
  event_reference_setup(conn, conn_params->connect_packet_timestamp,
                        conn_params->conn_req.win_offset);
  request_schedule(conn);

  return 0;

 err_put_devices:
  device_accessor_put(&conn->timer);

 err_put_radio:
  device_accessor_put(&conn->radio);

 err_out:
  return err;
}

static void ble_connection_closed(struct ble_connection_s *conn, uint8_t reason)
{

}

static KROUTINE_EXEC(connection_rq_done)
{
  struct ble_connection_s *conn = KROUTINE_CONTAINER(kr, *conn, request.base.kr);

  conn->flags &= ~F_REQ_SCHEDULED;

  if (conn->flags & F_SLAVE) {
    if (conn->request.data.packet_count) {
      if (!buffer_queue_isempty(&conn->reliable_queue)
          || !buffer_queue_isempty(&conn->bulk_queue)
          || !buffer_queue_isempty(&conn->request.data.tx_queue)
          || conn->request.data.md) {
        conn->event_progression = 1;
      } else {
        conn->event_progression *= 2;
        if (conn->event_progression > conn->slave_latency)
          conn->event_progression = conn->slave_latency + 1;
      }

      conn->last_event_counter = conn->next_event_counter;
      conn->last_unmapped_channel = conn->request.data.white_iv;
      conn->drops_at = conn->request.data.anchor + conn->timeout_ticks;
      conn->last_anchor = conn->request.data.anchor;
    } else {
      conn->event_progression++;
    }
  } else {
    conn->last_event_counter = conn->next_event_counter;
    conn->last_unmapped_channel = conn->request.data.white_iv;
    conn->drops_at = conn->request.data.anchor + conn->timeout_ticks;
    conn->last_anchor = conn->request.data.anchor;
  }

  request_schedule(conn);

  struct buffer_s *p;
  while ((p = buffer_queue_pop(&conn->request.data.rx_queue))) {
    struct net_task_s *task = net_scheduler_task_alloc(conn->scheduler);

    net_task_inbound_push(conn, conn, conn, conn->request.data.anchor, p);
    buffer_refdec(p);
  }
}

static
void llcp_handle(struct net_layer_s *conn, struct buffer_s *p)
{

}

static
void ble_connection_destroyed(struct net_layer_s *conn)
{
  device_stop(&conn->radio);
  device_accessor_put(&conn->timer);
  device_accessor_put(&conn->radio);
  net_layer_deinit(&conn->layer);
}

static
void ble_connection_task_handle(struct net_layer_s *conn,
                               struct net_task_s *task)
{
  switch (task->header.type) {
  case NET_TASK_INBOUND:
    if (task->inbound.source == conn) {
      switch (ble_data_llid_get(task->inbound.buffer)) {
      case BLE_LL_DATA_CONT:
      case BLE_LL_DATA_START:
        net_task_inbound_forward(task, conn->l2cap);
        return;

      case BLE_LL_CONTROL:
        llcp_handle(conn, task->inbound.buffer);
        goto cleanup;
      }
    } else if (task->inbound.source == layer->l2cap) {
      buffer_queue_pushback(&conn->reliable_queue, task->inbound.buffer);
    }
    goto cleanup;

  default:
  cleanup:
    net_task_cleanup(task);
  }
}

static
error_t ble_connection_bound(struct net_layer_s *conn,
                               void *addr,
                               struct net_layer_s *child)
{
  // Child list takes the reference. Dont double it.
  switch (layer->type) {
  case BLE_NET_L2CAP:
    conn->l2cap = child;
    return 0;

  case BLE_NET_SECURITY:
    conn->security = child;
    return 0;

  default:
    return -ENOTSUP;
  }
}

static
void ble_connection_unbound(struct net_layer_s *conn,
                              struct net_layer_s *child)
{
  if (child == conn->security)
    layer->security = NULL;

  if (child == conn->l2cap)
    layer->l2cap = NULL;
}

static const struct net_layer_handler_s connection_handler = {
  .destroyed = ble_connection_destroyed,
  .task_handle = ble_connection_task_handle,
  .bound = ble_connection_bound,
  .unbound = ble_connection_unbound,
};
