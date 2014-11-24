#ifndef SECURITY_DB_PRIV_H
#define SECURITY_DB_PRIV_H

enum key_handle_e
{
  KEY_HANDLE_IRK,
  KEY_HANDLE_LTK,
  KEY_HANDLE_DIV,
  KEY_HANDLE_CSRK,
};

void peer_subscribed_load(struct ble_security_db_s *db,
                          struct ble_peer_s *peer,
                          uint8_t offset);

error_t peer_subscribed_save(struct ble_security_db_s *db,
                             const struct ble_peer_s *peer,
                             uint8_t offset);

error_t ble_security_db_save(struct ble_security_db_s *db,
                             const struct ble_peer_s *peer);

error_t ble_peer_lookup_id(struct ble_security_db_s *db,
                           struct ble_peer_s *peer,
                           const uint8_t *random,
                           const uint16_t ediv);

error_t ble_security_db_next_id(struct ble_security_db_s *db,
                                uint64_t *value);

error_t ble_security_db_key_get(struct ble_security_db_s *db,
                                uint64_t did,
                                uint64_t key_handle,
                                uint8_t *out);

#endif
