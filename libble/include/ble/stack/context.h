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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_STACK_CONTEXT_H_
#define BLE_STACK_CONTEXT_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short BLE stack utilities context

   @this contains all declarations for the @ref {ble_stack_context_s}
   {library stack context} object.
*/

#include <net/scheduler.h>
#include <mutek/buffer_pool.h>
#include <ble/gattdb/db.h>
#include <ble/protocol/address.h>
#include <device/class/crypto.h>
#include <device/class/net.h>
#if defined(CONFIG_BLE_CRYPTO)
# include <ble/security_db.h>
#endif

/**
   @this is an object used as device context for other stack utilities
   like @ref {ble_peripheral_s} {peripheral} and @ref {ble_central_s}
   {central} contexts.

   The context object contains references to all necessary resources
   for stack utilities:
   @list
   @item network stack scheduler and packet allocator pool,
   @item radio device,
   @item a GATT Database,
   @item a random number generator and cryptography device.
   @end list
 */
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

struct persist_config;
struct persist_context_s;

/**
   @this initializes a stack context from relevant device paths.
 */
error_t ble_stack_context_init(struct ble_stack_context_s *ctx,
                               const char *ble_name,
                               const char *rtc_name,
                               const char *rng_name,
                               const char *sec_name,
                               struct persist_context_s *persist);

/**
   @this releases all stack context resources.
 */
void ble_stack_context_cleanup(struct ble_stack_context_s *ctx);

/**
   @this marks BLE device from the context as used
 */
void ble_stack_context_use(struct ble_stack_context_s *ctx);

/**
   @this marks BLE device from the context as unused
 */
void ble_stack_context_release(struct ble_stack_context_s *ctx);

/**
   @this generates a non-resolvable address.
 */
error_t ble_stack_context_address_non_resolvable_generate(struct ble_stack_context_s *ctx,
                                                          struct ble_addr_s *addr);

#if defined(CONFIG_BLE_SECURITY_DB)
/**
   @this generates a resolvable address using the device IRK from
   security database.
 */
error_t ble_stack_context_address_resolvable_generate(struct ble_stack_context_s *ctx,
                                                      struct ble_addr_s *addr);
#endif

/**
   @this retrieves device address from radio hardware.
 */
error_t ble_stack_context_local_address_get(struct ble_stack_context_s *ctx,
                                             struct ble_addr_s *addr);

/**
   @this generates a data connection access address.  Generated AA
   abides constraints from 6.B.2.1.2.
 */
uint32_t ble_stack_access_address_generate(struct ble_stack_context_s *ctx);

/**
   @this retrieves advertising data from GATT Database.  This will
   contain @ref {BLE_GATTDB_SERVICE_ADVERTISED} {advertised services},
   among other PDUs.
 */
void ble_stack_context_ad_collect(struct ble_stack_context_s *ctx,
                            uint8_t *ad, size_t ad_size_max,
                            size_t *ad_size_used);

#endif
