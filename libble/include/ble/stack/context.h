#ifndef BLE_STACK_CONTEXT_H_
#define BLE_STACK_CONTEXT_H_

#include <net/scheduler.h>
#include <ble/peer.h>
#include <mutek/buffer_pool.h>
#include <ble/gattdb/db.h>
#include <device/class/net.h>
#if defined(CONFIG_BLE_CRYPTO)
# include <ble/net/sm.h>
# include <ble/protocol/sm.h>
# include <ble/security_db.h>
# include <device/class/crypto.h>
#endif

struct ble_stack_context_s
{
  struct dev_rng_s rng;
#if defined(CONFIG_BLE_CRYPTO)
  struct ble_security_db_s security_db;
#endif
  struct device_net_s ble;
  struct net_scheduler_s scheduler;
  struct buffer_pool_s packet_pool;
  struct ble_gattdb_s gattdb;
  struct device_crypto_s crypto;
};

error_t ble_stack_context_init(struct ble_stack_context_s *ctx,
                               const char *ble_name,
                               const char *rtc_name,
                               const char *rng_name,
                               const char *sec_name,
                               const char *persist_name);

void ble_stack_context_cleanup(struct ble_stack_context_s *ctx);

error_t ble_stack_context_address_non_resolvable_generate(struct ble_stack_context_s *ctx,
                                                          struct ble_addr_s *addr);

error_t ble_stack_context_local_address_get(struct ble_stack_context_s *ctx,
                                             struct ble_addr_s *addr);

uint32_t ble_stack_access_address_generate(struct ble_stack_context_s *ctx);

void ble_stack_context_ad_collect(struct ble_stack_context_s *ctx,
                            uint8_t *ad, size_t ad_size_max,
                            size_t *ad_size_used);

struct ble_stack_connection_s;

struct ble_stack_connection_handler_s
{
#if defined(CONFIG_BLE_CRYPTO)
  void (*pairing_requested)(struct ble_stack_connection_s *conn,
                            bool_t bonding);
  void (*pairing_failed)(struct ble_stack_connection_s *conn,
                         enum sm_reason reason);
  void (*pairing_success)(struct ble_stack_connection_s *conn);
#endif
  void (*connection_closed)(struct ble_stack_connection_s *conn,
                            uint8_t reason);
};

struct ble_stack_context_handler_s
{
  void (*state_changed)(struct ble_stack_connection_s *conn,
                        bool_t connected);
};

struct ble_stack_connection_s
{
  const struct ble_stack_connection_handler_s *chandler;
  const struct ble_stack_context_handler_s *handler;
  struct ble_stack_context_s *context;
  struct net_layer_s *phy;
  struct net_layer_s *llcp;
#if defined(CONFIG_BLE_CRYPTO)
  struct net_layer_s *sm;
#endif
  struct ble_peer_s peer;
  dev_timer_value_t connection_tk;
};

struct ble_adv_connect_s;

error_t ble_stack_connection_create(struct ble_stack_connection_s *conn,
                                            struct ble_stack_context_s *context,
                                            const struct ble_stack_connection_handler_s *chandler,
                                            const struct ble_stack_context_handler_s *handler,
                                            bool_t is_master,
                                            const struct ble_adv_connect_s *conn_params,
                                            dev_timer_value_t anchor);

#if defined(CONFIG_BLE_CRYPTO)

void ble_stack_connection_pairing_request(struct ble_stack_connection_s *conn,
                                            bool_t mitm_protection,
                                            bool_t bonding);

void ble_stack_connection_pairing_accept(struct ble_stack_connection_s *conn,
                                           bool_t mitm_protection,
                                           uint32_t pin,
                                           const void *oob_data);

void ble_stack_connection_pairing_abort(struct ble_stack_connection_s *conn,
                                          enum sm_reason reason);

#endif

void ble_stack_connection_drop(struct ble_stack_connection_s *conn,
                                 uint8_t reason);

#endif
