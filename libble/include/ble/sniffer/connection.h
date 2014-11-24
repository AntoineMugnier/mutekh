#ifndef BLE_SNIFFER_CONNECTION_H
#define BLE_SNIFFER_CONNECTION_H

struct buffer_s;
struct ble_sniffer;

struct ble_connection_context
{
  uint8_t skd[16];
  uint8_t rand[8];
  uint16_t ediv;
};

enum ble_connection_drop_reason
{
  BLE_CONN_DROP_TIMEOUT,
  BLE_CONN_DROP_LATENCY,
  BLE_CONN_DROP_TERMINATED,
};

struct ble_connection_handler
{
  void (*event)(void *pvdata, uint16_t event, uint8_t channel, uint64_t timestamp, uint8_t packet_count);
  void (*master)(void *pvdata, uint16_t event, struct buffer_s *packet);
  void (*slave)(void *pvdata, uint16_t event, struct buffer_s *packet);
  void (*dropped)(void *pvdata, enum ble_connection_drop_reason reason);
  void (*enc_setup)(void *pvdata, const struct ble_connection_context *ctx, uint8_t *key);
};

void ble_connection_follow(
    void *pvdata,
    const struct ble_connection_handler *handler,
    struct ble_sniffer *radio,
    struct buffer_s *conn_packet);

#endif
