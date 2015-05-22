










#if 0

static void connection_channel_setup(
    struct ble_connection *conn,
    struct channel_setup *setup,
    uint16_t event_count,
    bool_t from_master)
{
    setup->access = conn->conn_params.access_address;
    setup->crc_init = conn->conn_params.crc_init;
    setup->channel = conn->connection.remapped_channel[
        conn->connection.event_unmapped_channel];
    setup->white_iv = setup->channel;
}

/*
  Schedule for next packet, two things to do:
  - set next channel to listen on
  - set deadline to consider a non-event
*/
static void connection_rx_next_prepare(struct ble_connection *conn)
{
    struct channel_setup setup;
    uint64_t next_deadline = 0;

    switch (conn->state) {
    case ADV_FIRST_WAIT:
        connection_adv_channel_setup(conn, &setup);
        next_deadline = connection_time_get(conn) + 200000;
        conn->adv_timeout = 20;
        break;

    case ADV_WAIT_EVENT:
        connection_adv_channel_setup(conn, &setup);
        next_deadline = conn->last_adv_event_ts + 200000;
        break;

    case ADV_WAIT_HOP:
        connection_adv_channel_setup(conn, &setup);
        next_deadline = conn->last_adv_event_ts + 10000;
        break;

    case ADV_WAIT_REPLY:
        connection_adv_channel_setup(conn, &setup);
        next_deadline = conn->last_adv_ts + T_ADV_UNIT;
        break;

    case CONN_WAIT_START:
        connection_channel_setup(conn, &setup, conn->connection.event_count, 1);
        next_deadline = conn->connection.last_event_ts
            + conn->connection.window_size
            + conn->connection.accumulated_window_widening
            + T_CONN_UNIT / 2;
        break;

    case CONN_WAIT_HOP:
    case CONN_WAIT_EVENT:
    case CONN_WAIT_DATA:
        connection_channel_setup(conn, &setup, conn->connection.event_count,
                                   !(conn->connection.event_packet_count & 1));
        next_deadline = conn->connection.last_event_ts
            + conn->connection.interval_size
            - conn->connection.accumulated_window_widening - T_CONN_UNIT / 2 - T_SETUP_LATENCY * 3;
        break;
    }

    connection_rx_packet_setup(conn, &setup);

    uint64_t now = connection_time_get(conn);

    next_deadline = __MAX(now + 20, next_deadline);
    nrf_reg_set(conn->timer, NRF51_TIMER_CC(TIMER_DEADLINE), (uint32_t)next_deadline);
    nrf_it_enable(conn->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE));
}

/*
  After a packet arrived, things to do:
  - update state
  - update counters / timers
  - update event counts
 */
static void connection_packet_handle(
    struct ble_connection *conn,
    const struct packet *packet)
{
    struct ble_connection_event *event;

    switch (conn->state) {
    case ADV_FIRST_WAIT:
        switch (packet_ind_type_get(packet)) {
        case ADV_IND:
        case ADV_NONCONN_IND:
        case ADV_DIRECT_IND:
            conn->state = ADV_WAIT_EVENT;
            conn->last_adv_event_ts
                = conn->last_adv_ts
                = packet->timestamp;
            conn->adv_count_todo = 2 << LOWPASS_LOG2;
            conn->adv_timeout = 40;
            conn->adv_first_after_init = 1;
            break;

        case CONNECT_REQ:
            goto connect;

        default:
            break;
        }
        break;

    case ADV_WAIT_EVENT:
        if (conn->adv_first_after_init) {
            lowpass_value_reset(&conn->adv_interval,
                                packet->timestamp - conn->last_adv_event_ts);
        }
        conn->adv_first_after_init = 0;

        switch (packet_ind_type_get(packet)) {
        case ADV_NONCONN_IND:
        case ADV_DIRECT_IND:
        case ADV_IND:
            lowpass_value_accumulate(&conn->adv_interval,
                                     packet->timestamp - conn->last_adv_event_ts);
            conn->last_adv_event_ts
                = conn->last_adv_ts
                = packet->timestamp;
            if (conn->adv_count_todo == 1) {
                struct ble_connection_event *event = slab_alloc(&conn->event_pool);
                event->type = EVENT_ADV_STATS;
                event->adv_stats.event_interval = lowpass_value_get(&conn->adv_interval);
                event->adv_stats.adv_interval = lowpass_value_get(&conn->adv_ind_interval);
                connection_event_push(conn, event);
            }

            if (conn->adv_count_todo)
                conn->adv_count_todo--;
            break;
        default:
            break;
        }
        // fallthrough
    case ADV_WAIT_HOP:
        switch (packet_ind_type_get(packet)) {
        case ADV_NONCONN_IND:
        case ADV_DIRECT_IND:
        case ADV_IND:
            if (conn->last_adv_channel == packet->channel - 1)
                lowpass_value_accumulate(&conn->adv_ind_interval,
                                         packet->timestamp - conn->last_adv_ts);
            conn->last_adv_ts = packet->timestamp;
            conn->last_adv_channel = packet->channel;
            break;
        default:
            break;
        }
        // fallthrough
    case ADV_WAIT_REPLY:
        conn->state = ADV_WAIT_REPLY;

        if (packet_ind_type_get(packet) == CONNECT_REQ)
            goto connect;

        break;

    case CONN_WAIT_HOP:
        break;

    case CONN_WAIT_START:
        conn->connection.latency = 0;

    case CONN_WAIT_EVENT:
        conn->connection.last_potential_event_ts = packet->timestamp;
    case CONN_WAIT_DATA:
        if (conn->connection.event_packet_count == 1) {
            conn->connection.last_event_ts = conn->connection.last_potential_event_ts;
            conn->connection.since_last_event = 0;
            conn->connection.accumulated_window_widening = 0;
        }

        conn->state = CONN_WAIT_DATA;
        conn->connection.event_packet_count++;

        if (/*!(conn->connection.event_packet_count & 1)
              && */packet_data_llid_get(packet) == LLID_CONTROL) {
            switch (packet_data_control_get(packet)) {
            case LL_ENC_REQ:
                // Packet from master
                memcpy(conn->connection.crypto_rand, packet->data + 3, 8);
                memcpy(conn->connection.crypto_ediv, packet->data + 11, 2);
                memcpy(conn->connection.crypto_skd, packet->data + 13, 8);
                memcpy(conn->connection.crypto_iv, packet->data + 21, 4);
                break;

            case LL_ENC_RSP:
                // Packet from slave
                memcpy(conn->connection.crypto_skd + 8, packet->data + 3, 8);
                memcpy(conn->connection.crypto_iv + 4, packet->data + 11, 4);
                break;

            case LL_START_ENC_REQ:
                // Packet from slave
                conn->connection.master_crypto.last_sn = !(packet->data[0] & 0x4);
                conn->connection.slave_crypto.last_sn = !!(packet->data[0] & 0x8);

                conn->connection.crypto_enabled = 1;

                {
                    struct ble_connection_event *event;

                    event = slab_alloc(&conn->event_pool);
                    event->type = EVENT_CONNECTION_ENCRYPT;
                    event->event_count = conn->conn_params_update.instant;
                    memcpy(event->encrypt.stk, conn->connection.crypto_key, 16);
                    memcpy(event->encrypt.skd, conn->connection.crypto_skd, 16);
                    memcpy(event->encrypt.key, conn->connection.master_crypto.key, 16);
                    memcpy(event->encrypt.iv, conn->connection.crypto_iv, 8);
                    connection_event_push(conn, event);
                }

                break;

            case LL_START_ENC_RSP:
                // Packet from master
                break;

            case LL_CONNECTION_UPDATE_REQ:
                packet_conn_params_update_parse(packet, &conn->conn_params_update);
                conn->conn_param_update_pending = 1;

                {
                    struct ble_connection_event *event;

                    event = slab_alloc(&conn->event_pool);
                    event->type = EVENT_CONNECTION_UPDATE;
                    event->connection_update = conn->conn_params_update;
                    event->event_count = conn->conn_params_update.instant;
                    connection_event_push(conn, event);
                }

                break;

            case LL_CHANNEL_MAP_REQ:
                conn->conn_channel_map_pending = 1;
                conn->conn_next_channel_map = endian_le32_na_load(packet->data + 3);
                conn->conn_next_channel_map |= (uint64_t)(packet->data[7] & 0x1f) << 32;
                conn->conn_channel_map_instant = endian_le16_na_load(packet->data + 8);

                {
                    struct ble_connection_event *event;

                    event = slab_alloc(&conn->event_pool);
                    event->type = EVENT_CHANNEL_MAP_UPDATE;
                    event->channel_map_update = conn->conn_next_channel_map;
                    event->event_count = conn->conn_channel_map_instant;
                    connection_event_push(conn, event);
                }

                break;

            default:
                break;
            }
        }

        break;

    connect:
        packet_conn_param_parse(packet, &conn->conn_params);
        conn->connection.transmit_window_size
            = T_CONN_UNIT * conn->conn_params.win_size;
        conn->connection.interval_size
            = T_CONN_UNIT * conn->conn_params.interval;
        conn->connection.window_widening_size
            = T_CONN_UNIT * conn->conn_params.interval
            * (sca_wc[conn->conn_params.sca] + 100) / 1000000;
        conn->connection.timeout
            = 10000 * conn->conn_params.timeout;
        conn->connection.event_count = 0;
        conn->connection.latency = 0;
        conn->connection.event_unmapped_channel = conn->conn_params.hop;
        channel_map_update(conn);

        // Pretend we have a previous conn event available as reference
        conn->connection.last_event_ts
            = packet->timestamp + 320 // End of CONN_REQ packet
            + T_CONN_UNIT * conn->conn_params.win_offset; // Offset
        conn->connection.since_last_event = 0;
        conn->connection.accumulated_window_widening = 0;

        conn->state = CONN_WAIT_START;
        conn->connection.event_packet_count = 0;

        event = slab_alloc(&conn->event_pool);
        event->type = EVENT_CONNECTION;
        event->connection = conn->conn_params;
        connection_event_push(conn, event);
    }
}

            conn->connection.last_event_ts += conn->connection.interval_size;
            conn->connection.since_last_event += conn->connection.interval_size;
            conn->connection.accumulated_window_widening += conn->connection.window_widening_size;
            conn->connection.event_unmapped_channel
                = (conn->connection.event_unmapped_channel
                   + conn->conn_params.hop) % 37;
            conn->connection.event_count++;
            conn->state = CONN_WAIT_EVENT;

            if (conn->conn_param_update_pending
                && conn->conn_params_update.instant == conn->connection.event_count) {

                conn->conn_param_update_pending = 0;
//                struct ble_connection_event *event;
//
//                event = slab_alloc(&conn->event_pool);
//                event->type = EVENT_CONNECTION_UPDATE;
//                event->connection_update = conn->conn_params_update;
//                event->event_count = conn->conn_params_update.instant;
//                connection_event_push(conn, event);

                conn->conn_params.win_size = conn->conn_params_update.win_size;
                conn->conn_params.win_offset = conn->conn_params_update.win_offset;
                conn->conn_params.interval = conn->conn_params_update.interval;
                conn->conn_params.timeout = conn->conn_params_update.timeout;
                conn->conn_params.latency = conn->conn_params_update.latency;

                conn->connection.transmit_window_size
                    = T_CONN_UNIT * conn->conn_params.win_size;
                conn->connection.interval_size
                    = T_CONN_UNIT * conn->conn_params.interval;
                conn->connection.window_widening_size
                    = T_CONN_UNIT * conn->conn_params.interval
                    * (sca_wc[conn->conn_params.sca] + 50) / 1000000;
                conn->connection.timeout
                    = 10000 * conn->conn_params.timeout;

                conn->connection.last_event_ts
                    += T_CONN_UNIT * conn->conn_params.win_offset;

                conn->state = CONN_WAIT_START;
            }

            if (conn->conn_channel_map_pending
                && conn->conn_channel_map_instant == conn->connection.event_count) {
//                struct ble_connection_event *event;
//
//                event = slab_alloc(&conn->event_pool);
//                event->type = EVENT_CHANNEL_MAP_UPDATE;
//                event->channel_map_update = conn->conn_next_channel_map;
//                event->event_count = conn->conn_channel_map_instant;
//                connection_event_push(conn, event);

                conn->conn_channel_map_pending = 0;
                conn->conn_params.channel_map = conn->conn_next_channel_map;
                channel_map_update(conn);
            }

            /* event = slab_alloc(&conn->event_pool); */
            /* event->type = EVENT_HOP; */
            /* event->hop.channel = conn->connection.event_unmapped_channel; */
            /* event->hop.mapped_channel = channel_remap( */
            /*     conn->conn_params.channel_map, */
            /*     conn->conn_params.channel_count, */
            /*     conn->connection.event_unmapped_channel); */
            /* event->hop.packet_count = conn->connection.event_packet_count; */
            /* connection_event_push(conn, event); */

        }
    }
}

static void connection_event_push(
    struct ble_connection *conn,
    struct ble_connection_event *event)
{
    switch (event->type) {
    case EVENT_PACKET_ADV:
    case EVENT_PACKET_DATA_MS:
    case EVENT_PACKET_DATA_SM:
        event->timestamp = event->packet.packet->timestamp;
        break;
    default:
        event->timestamp = connection_time_get(conn);
        break;
    }

    connection_event_queue_pushback(&conn->event_queue, event);
    semaphore_give(&conn->event_queue_sem, 1);
}

#endif
