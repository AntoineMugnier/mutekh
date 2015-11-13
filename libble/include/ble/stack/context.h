#ifndef BLE_STACK_CONTEXT_H_
#define BLE_STACK_CONTEXT_H_

#include <net/scheduler.h>
#include <device/class/crypto.h>
#include <ble/security_db.h>
#include <mutek/buffer_pool.h>
#include <ble/gattdb/db.h>
#include <device/class/crypto.h>
#include <device/class/net.h>

struct ble_stack_context_s
{
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s rng;
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

#endif
