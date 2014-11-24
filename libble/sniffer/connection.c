#include <hexo/types.h>

#include <mutek/kroutine.h>
#include <mutek/printk.h>
#include <mutek/semaphore.h>
#include <mutek/thread.h>
#include <mutek/slab.h>
#include <mutek/buffer_pool.h>

#include <stdlib.h>

#include <ble/protocol/address.h>
#include <ble/sniffer/connection.h>
#include <ble/crypto.h>
#include <ble/protocol/data.h>
#include <ble/sniffer/radio.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/advertise.h>

struct ble_connection;
struct conn_side;
struct event;

#define T_CONN_PACKET_LENGTH 320

static KROUTINE_EXEC(conn_done);

static void connection_event_done(
    struct ble_connection *conn,
    struct buffer_s *first);
static void connection_blank_event_done(
    struct ble_connection *conn);
static void connection_event_next_schedule(
    struct ble_connection *conn);
static void conn_control_packet_process(
    struct conn_side *side,
    struct event *event,
    struct buffer_s *packet);
static struct buffer_s *event_packet_decrypt(
    struct ble_connection *conn,
    struct event *event,
    struct buffer_s *ciphered);

#define GCT_CONTAINER_ALGO_event_list CLIST

struct event
{
  GCT_CONTAINER_ENTRY(event_list, entry);
  uint8_t channel;
  uint16_t event;
  uint16_t packet_count;
  uint64_t timestamp;
  buffer_queue_root_t packet_queue;
};

GCT_CONTAINER_TYPES(event_list, struct event *, entry);
GCT_CONTAINER_FCNS(event_list, inline, event_list,
                   init, destroy, pushback, pop, remove, isempty, head);

struct conn_side
{
  bool_t master_to_slave;

  struct ble_addr_s addr;

  uint32_t packet_counter;

  bool_t last_sn;

  struct buffer_s *last_clear_packet;
  uint32_t last_cipher_packet_crc;
};

enum side
{
    SIDE_MASTER, SIDE_SLAVE, SIDE_COUNT
};

struct params {
    uint64_t window_us;
    uint64_t interval_us;
    uint32_t timeout_us;
    uint32_t access_address;
    uint32_t crc_init;
    uint32_t slave_latency;
    uint8_t hop;
    uint8_t sca;
    uint8_t remapped_freq[37];
};

struct ble_connection
{
  struct ble_sniffer_request req;

  struct event *event;

  bool_t crypto_enabled;
  struct ble_connection_context ctx;
  uint8_t key[16];
  uint8_t iv[8];

  struct slab_s event_pool;
  event_list_root_t event_list;
  struct semaphore_s event_sem;

  void *pvdata;
  const struct ble_connection_handler *handler;
  struct ble_sniffer *radio;

  struct params params;

  struct buffer_s *conn_params_pending;
  struct buffer_s *channel_map_pending;

  struct conn_side side[SIDE_COUNT];

  int64_t last_event_anchor;

  int64_t since_last_anchor;
  int64_t window_widening;

  int64_t drops_at;

  uint16_t event_count;
  uint8_t unmapped_channel;

  bool_t running;
  enum ble_connection_drop_reason drop_reason;
};

STRUCT_COMPOSE(ble_connection, req);

ALWAYS_INLINE
struct ble_connection *connection_from_side(
    struct conn_side *side)
{
    if (side->master_to_slave)
        return KROUTINE_CONTAINER(side, struct ble_connection, side[0]);
    else
        return KROUTINE_CONTAINER(side, struct ble_connection, side[1]);
}

static int8_t channel_remap(uint64_t map, uint8_t channel_count, int8_t channel)
{
    if (map & (1ULL << channel))
        return channel;

    channel %= channel_count;

    int8_t current = -1;
    bool_t active = map & 1;
    uint64_t transitions = map ^ (map >> 1);

    while (transitions && channel > current) {
        uint8_t next_change = __builtin_ctzll(transitions);
        transitions ^= 1ULL << next_change;

        if (!active) {
            active = 1;
            channel += next_change - current;
        } else {
            active = 0;
        }

        current = next_change;
    }

    return channel;
}

static void connection_params_apply(struct params *cp, const struct buffer_s *packet)
{
    const uint8_t *data = packet->data + packet->begin;

    uint64_t channel_map = endian_le32_na_load(data + 30)
        | ((uint64_t)(data[34] & 0x1f) << 32);
    uint8_t channel_count = __builtin_popcountll(channel_map);

    cp->sca = data[35] >> 5;

    cp->access_address = endian_le32_na_load(data + 14);
    cp->crc_init = endian_le32_na_load(data + 18) & 0xffffff;

    for (uint8_t i = 0; i < 37; ++i)
        cp->remapped_freq[i] = ble_channel_freq_mhz(
            channel_remap(channel_map, channel_count, i)) - 2400;

    cp->window_us = BLE_T_CONN_UNIT * data[21];
    cp->interval_us = BLE_T_CONN_UNIT * endian_le16_na_load(data + 24);
    cp->slave_latency = endian_le16_na_load(data + 26);
    cp->timeout_us = 10000 * endian_le16_na_load(data + 28);
    cp->hop = data[35] & 0x1f;
}

static void connection_params_update_apply(struct params *cp, const struct buffer_s *packet)
{
    const uint8_t *data = packet->data + packet->begin;

    cp->window_us = BLE_T_CONN_UNIT * data[3];
    cp->interval_us = BLE_T_CONN_UNIT * endian_le16_na_load(data + 6);
    cp->slave_latency = endian_le16_na_load(data + 8);
    cp->timeout_us = 10000 * endian_le16_na_load(data + 10);
}

static void connection_params_update_check(struct ble_connection *conn)
{
    if (!conn->conn_params_pending)
        return;

    const uint8_t *data = conn->conn_params_pending->data
        + conn->conn_params_pending->begin;

    uint16_t instant = endian_le16_na_load(data + 12);

    if (instant != conn->event_count)
        return;

    uint16_t win_offset = endian_le16_na_load(data + 4);

    connection_params_update_apply(&conn->params, conn->conn_params_pending);
    conn->since_last_anchor += BLE_T_CONN_UNIT * win_offset;

    buffer_refdec(conn->conn_params_pending);
    conn->conn_params_pending = NULL;
}

static void connection_channel_map_update_check(struct ble_connection *conn)
{
    if (!conn->channel_map_pending)
        return;

    const uint8_t *data = conn->channel_map_pending->data
        + conn->channel_map_pending->begin;

    uint16_t instant = endian_le16_na_load(data + 8);

    if (instant != conn->event_count)
        return;

    uint64_t channel_map = endian_le32_na_load(data + 3)
        | ((uint64_t)(data[7] & 0x1f) << 32);
    uint8_t channel_count = __builtin_popcountll(channel_map);

    for (uint8_t i = 0; i < 37; ++i)
        conn->params.remapped_freq[i] = ble_channel_freq_mhz(
            channel_remap(channel_map, channel_count, i)) - 2400;

    buffer_refdec(conn->channel_map_pending);
    conn->channel_map_pending = NULL;
}

static CONTEXT_ENTRY(conn_thread)
{
    struct ble_connection *conn = param;
    struct event *event;
    struct buffer_s *packet;
    //    uint64_t next_packet = 0;
    //    uint64_t next_event = 0;

    while (conn->running) {
        semaphore_take(&conn->event_sem, 1);

        event = event_list_pop(&conn->event_list);

        /* printk("Connection event %d, %d packets\n", event->event, event->packet_count); */

        conn->handler->event(conn->pvdata, event->event, event->channel,
                             event->timestamp, event->packet_count);

        while ((packet = buffer_queue_pop(&event->packet_queue))) {
          struct conn_side *side = &conn->side[packet->index & 1];

          struct buffer_s *cleartext = event_packet_decrypt(conn, event, packet);
          buffer_refdec(packet);

          conn_control_packet_process(side, event, cleartext);

          if (packet->index & 1)
            conn->handler->slave(conn->pvdata, event->event, cleartext);
          else
            conn->handler->master(conn->pvdata, event->event, cleartext);

          buffer_refdec(cleartext);
        }

        slab_free(&conn->event_pool, event);
    }

    conn->handler->dropped(conn->pvdata, conn->drop_reason);

    if (conn->channel_map_pending)
      buffer_refdec(conn->channel_map_pending);
    if (conn->conn_params_pending)
      buffer_refdec(conn->conn_params_pending);

    if (conn->side[0].last_clear_packet)
      buffer_refdec(conn->side[0].last_clear_packet);
    if (conn->side[1].last_clear_packet)
      buffer_refdec(conn->side[1].last_clear_packet);

    free(conn);
}

static KROUTINE_EXEC(conn_done)
{
    struct ble_connection *conn = KROUTINE_CONTAINER(kr, *conn, req.base.kr);
    struct buffer_s *packet;
    struct event *event = conn->event;
    conn->event = NULL;

    buffer_queue_init(&event->packet_queue);
    event->packet_count = conn->req.packet_count;

    while ((packet = buffer_queue_pop(&conn->req.queue))) {
      buffer_queue_pushback(&event->packet_queue, packet);
      buffer_refdec(packet);
    }

    event_list_pushback(&conn->event_list, event);
    semaphore_give(&conn->event_sem, 1);

    if (conn->req.packet_count > 1) {
      packet = buffer_queue_head(&event->packet_queue);

      // Dont loose time before rescheduling of next event

      connection_event_done(conn, packet);

      buffer_refdec(packet);
    } else {
        connection_blank_event_done(conn);
    }
}

static
void connection_blank_event_done(struct ble_connection *conn)
{
  if (conn->drops_at < ble_sniffer_time_get(conn->radio)) {
    conn->drop_reason = BLE_CONN_DROP_TIMEOUT;
    conn->running = 0;
    return;
  }

  connection_event_next_schedule(conn);
}

static
void connection_event_done(struct ble_connection *conn, struct buffer_s *first)
{
    conn->last_event_anchor = first->timestamp;
    conn->since_last_anchor = 0;
    conn->drops_at = first->timestamp + conn->params.timeout_us;

    connection_event_next_schedule(conn);
}

static
void connection_event_next_schedule(struct ble_connection *conn)
{
    conn->since_last_anchor += conn->params.interval_us;
    conn->event_count++;

    connection_params_update_check(conn);
    connection_channel_map_update_check(conn);

    conn->unmapped_channel += conn->params.hop;
    if (conn->unmapped_channel >= 37)
        conn->unmapped_channel -= 37;

    assert(conn->since_last_anchor > 0);

    uint32_t window_widening = conn->since_last_anchor
      * (ble_sca_wc[conn->params.sca] + 100)
      / 1000000;

    conn->event = slab_alloc(&conn->event_pool);

    conn->event->channel = conn->unmapped_channel;
    conn->event->event = conn->event_count;
    conn->event->timestamp
      = conn->last_event_anchor
      + conn->since_last_anchor;

    conn->req.start = conn->last_event_anchor
      + conn->since_last_anchor
      - window_widening
      - BLE_T_CONN_UNIT;
    conn->req.window = conn->params.window_us;
    conn->req.initial_timeout = conn->params.window_us
      + window_widening * 2
      + BLE_T_CONN_UNIT;
    conn->req.access = conn->params.access_address;
    conn->req.crc_init = conn->params.crc_init;
    conn->req.frequency = conn->params.remapped_freq[conn->unmapped_channel] + 2400;
    conn->req.white_iv = conn->unmapped_channel;
    conn->req.rx_packet_max = 0;
    conn->req.chain_mode = 0;
    conn->req.filter = 0;
    conn->req.t_ifs_max = 150;

    ble_sniffer_request(conn->radio, &conn->req, NULL);
}

static struct buffer_s *event_packet_decrypt(
    struct ble_connection *conn,
    struct event *event,
    struct buffer_s *packet)
{
    struct conn_side *side = &conn->side[packet->index & 1];
    bool_t sn = ble_data_sn_get(packet);

    if (!conn->crypto_enabled || packet->end - packet->begin <= 2)
      return buffer_refinc(packet);

    if (side->last_clear_packet
        && side->last_sn == sn
        && packet->crc_valid
        && side->last_cipher_packet_crc == packet->crc) {
      struct buffer_s *clear = side->last_clear_packet;

      clear->timestamp = packet->timestamp;
      clear->data[clear->begin] = packet->data[packet->begin];
      clear->data[clear->begin + 1] = packet->data[packet->begin + 1];
      clear->duration = packet->duration;
      clear->rssi = packet->rssi;
      clear->index = packet->index;

      return buffer_refinc(clear);
    }

    if (packet->crc_valid)
      side->last_sn = sn;
    else {
      if (side->last_sn != sn)
        side->packet_counter++;

      return buffer_refinc(packet);
    }

    struct buffer_s *clear = ble_sniffer_packet_alloc(conn->radio);

    assert(clear);

    packet->ciphered = 1;
    packet->decrypted = 0;
    packet->mic_valid = 0;

    /* printk("\n"); */
    /* printk("Crypted: [%P]\n", packet->data + packet->begin, packet->end - packet->begin); */
    /* printk("   from master %d\n", !(packet->index & 1)); */
    /* printk("   key [%P]\n", conn->key, 16); */
    /* printk("   iv [%P]\n", conn->iv, 8); */

    for (uint32_t count = side->packet_counter + 1; count < side->packet_counter + 15; ++count) {
      error_t err = ble_ccm_decrypt(conn->key, conn->iv, count, !(packet->index & 1), packet, clear);

      /* printk("   Try with count = %d: %d\n", count, err); */

      switch (err) {
      case -EIO:
        goto fail;

      case -EINVAL:
        continue;

      case 0:
        /* printk("   Clear: [%P]\n", clear->data + clear->begin, clear->end - clear->begin); */
        side->packet_counter = count;
        goto ok;
      }
    }
    goto fail;
 ok:

    clear->ciphered = 1;
    clear->decrypted = 1;
    clear->mic_valid = 1;

    if (side->last_clear_packet)
      buffer_refdec(side->last_clear_packet);
    side->last_clear_packet = buffer_refinc(clear);
    side->last_cipher_packet_crc = packet->crc;

    return clear;

 fail:
    buffer_refdec(clear);

    return buffer_refinc(packet);
}

static
void conn_control_packet_process(struct conn_side *side, struct event *event, struct buffer_s *packet)
{
    struct ble_connection *conn = connection_from_side(side);
    const uint8_t *data = packet->data + packet->begin;

    if (!packet->crc_valid)
      return;

    switch (ble_data_llid_get(packet)) {
    case BLE_LL_RESERVED:
    case BLE_LL_DATA_CONT:
    case BLE_LL_DATA_START:
        break;

    case BLE_LL_CONTROL:
        switch (ble_data_control_get(packet)) {
        case BLE_LL_CONNECTION_UPDATE_REQ:
            if (conn->conn_params_pending)
                buffer_refdec(conn->conn_params_pending);
            conn->conn_params_pending = buffer_refinc(packet);
            break;

        case BLE_LL_CHANNEL_MAP_REQ:
            if (conn->channel_map_pending)
                buffer_refdec(conn->channel_map_pending);
            conn->channel_map_pending = buffer_refinc(packet);
            break;

        case BLE_LL_TERMINATE_IND:
          conn->drop_reason = BLE_CONN_DROP_TERMINATED;
            conn->running = 0;
            break;

        case BLE_LL_ENC_REQ:
          memcpy(conn->ctx.rand, data + 3, 8);
          conn->ctx.ediv = endian_le16_na_load(data + 11);
          memcpy(conn->ctx.skd, data + 13, 8);
          memcpy(conn->iv, data + 21, 4);
          break;

        case BLE_LL_ENC_RSP:
          memcpy(conn->ctx.skd + 8, data + 3, 8);
          memcpy(conn->iv + 4, data + 11, 4);

          conn->handler->enc_setup(conn->pvdata, &conn->ctx, conn->key);
          // Assume handler populated key

          conn->side[0].packet_counter = -1;
          conn->side[1].packet_counter = -1;

          break;

        case BLE_LL_START_ENC_REQ:
          conn->crypto_enabled = 1;
          break;

        case BLE_LL_PAUSE_ENC_REQ:
          conn->crypto_enabled = 0;
          break;

        default:
          break;
        }
    }
}


static SLAB_GROW(event_pool_grow)
{
    return 10;
}

void ble_connection_follow(
    void *pvdata,
    const struct ble_connection_handler *handler,
    struct ble_sniffer *radio,
    struct buffer_s *conn_packet)
{
    struct ble_connection *conn = malloc(sizeof(*conn));

    assert(conn);

    memset(conn, 0, sizeof(*conn));

    conn->running = 1;

    kroutine_init(&conn->req.base.kr, conn_done, KROUTINE_INTERRUPTIBLE);

    conn->handler = handler;
    conn->pvdata = pvdata;
    conn->radio = radio;

    slab_init(&conn->event_pool, sizeof(struct event), event_pool_grow, mem_scope_sys);
    event_list_init(&conn->event_list);
    semaphore_init(&conn->event_sem, 0);

    struct thread_attr_s attr = {
        .stack_size = 1024,
    };
    thread_create(conn_thread, conn, &attr);

    const uint8_t *data = conn_packet->data + conn_packet->begin;

    ble_advertise_packet_txaddr_get(conn_packet, &conn->side[SIDE_MASTER].addr);
    ble_advertise_packet_rxaddr_get(conn_packet, &conn->side[SIDE_SLAVE].addr);

    connection_params_apply(&conn->params, conn_packet);

    conn->since_last_anchor = BLE_T_CONN_UNIT * (endian_le16_na_load(data + 22) + 1)
      - conn->params.interval_us;
    conn->last_event_anchor = conn_packet->timestamp + conn_packet->duration;
    conn->drops_at = conn->last_event_anchor
      + conn->since_last_anchor
      + 7 * conn->params.interval_us;
    conn->event_count = 0xffff;
    conn->unmapped_channel = 0;

    conn->side[SIDE_MASTER].master_to_slave = 1;
    conn->side[0].last_clear_packet = NULL;
    conn->side[1].last_clear_packet = NULL;
    connection_event_next_schedule(conn);
}
