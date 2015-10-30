
/**
 BLE Attribute protocol layer.
 */
struct ble_att_s
{
  struct net_layer_s layer;

  struct net_layer_s *server;
  struct net_layer_s *client;

  net_task_queue_root_t transaction_queue;
  struct ble_att_transaction_s *transaction_pending;

  uint16_t server_mtu;
  uint16_t mtu;
};


struct ble_att_transaction_s *att_request_allocate(struct ble_att_s *att,
                                                   size_t total_size);

error_t att_error_serialize(struct buffer_s *p,
                            uint8_t command,
                            uint16_t handle,
                            uint8_t error);

error_t att_response_serialize(struct buffer_s *p,
                               const struct ble_att_transaction_s *txn,
                               uint16_t mtu);

error_t att_request_serialize(struct buffer_s *p,
                              const struct ble_att_transaction_s *txn,
                              uint16_t mtu);

enum ble_att_error_e att_request_parse(struct ble_att_s *att,
                                       const struct buffer_s *p,
                                       struct ble_att_transaction_s **rtxn);

enum ble_att_error_e att_response_parse(struct ble_att_s *att,
                                        const struct buffer_s *p,
                                        struct ble_att_transaction_s **rtxn);
