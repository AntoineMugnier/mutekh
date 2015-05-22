#ifndef BLE_NET_CONNECTION_H_
#define BLE_NET_CONNECTION_H_

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>
#include <ble/protocol/error.h>
#include <ble/protocol/data.h>

struct buffer_s;

#define BLE_LAYER_TYPE_CONNECTION NET_LAYER_TYPE(BleC)

/**
 BLE Connection asynchronous data layer.

 Handles connection data stream.

 Internally, this layer handles ACK-C PDUs (Connection control), it
 requires registration of a L2CAP layer above to handle packet
 fragmentation and L2CAP demux.
 */
struct ble_connection_s
{
  struct net_layer_s layer;

  struct dev_ble_radio_s radio;
  struct dev_timer_s timer;

  struct net_layer_s *l2cap;
  struct net_layer_s *security;

  struct dev_ble_radio_rq_s request;

  buffer_queue_root_t reliable_queue;
  buffer_queue_root_t bulk_queue;

  dev_timer_value_t drops_at;

  dev_timer_value_t last_anchor;
  dev_timer_delay_t timeout_ticks;
  dev_timer_delay_t interval_us;
  dev_timer_delay_t interval_ticks;
  dev_timer_delay_t window_us;
  dev_timer_delay_t window_ticks;

  uint16_t last_event_counter;
  uint16_t next_event_counter;
  uint16_t slave_latency;
  uint16_t event_progression;

  uint8_t mapped_channel[37];
  uint8_t hop;
  uint8_t sca_ppm; // Only if slave
  uint8_t last_unmapped_channel;

  uint8_t flags;

  struct buffer_s *conn_params_update;
  struct buffer_s *channel_map_update;
  uint16_t conn_params_update_instant;
  uint16_t channel_map_update_instant;
};

struct ble_connection_parameters_s
{
  struct ble_adv_connect_s conn_req;
  dev_timer_value_t connect_packet_timestamp;
  bool_t is_master;
};

STRUCT_INHERIT(ble_connection_s, net_layer_s, layer);

struct ble_connection_handler_s
{
  void (*closed)(
    struct ble_connection_s *conn,
    uint8_t reason);
};

error_t ble_connection_init(
  struct ble_connection_s *connection,
  const struct ble_connection_handler_s *handler,
  struct net_scheduler_s *scheduler,
  const char *radio_dev,
  const struct ble_connection_parameters_s *conn_params);

#endif
