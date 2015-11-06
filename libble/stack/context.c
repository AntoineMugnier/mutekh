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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <ble/stack/context.h>

#include <mutek/printk.h>

static SLAB_GROW(packet_pool_grow)
{
    return 40;
}

error_t ble_stack_context_init(struct ble_stack_context_s *ctx,
                               const char *ble_name,
                               const char *rtc_name,
                               const char *rng_name,
                               const char *sec_name,
                               const char *persist_name)
{
  error_t err;
#if defined(CONFIG_BLE_CRYPTO)
  struct dev_rng_s rng;

  err = dev_rng_init(&ctx->rng, sec_name);
  if (err) {
    printk("Error while initing DRBG: %d\n", err);
    return err;
  }

  err = dev_rng_init(&rng, rng_name);
  if (err) {
    printk("Error while initing RNG: %d\n", err);
    goto rng_close;
  }

  err = dev_rng_wait_seed_from_other(&ctx->rng, &rng, 16);
  if (err) {
    printk("Error while seeding from RNG: %d\n", err);
    goto rng_close2;
  }

  dev_rng_cleanup(&rng);

  err = device_get_accessor_by_path(&ctx->crypto, NULL, sec_name, DRIVER_CLASS_CRYPTO);
  if (err) {
    printk("Error while opening crypto device: %d\n", err);
    goto rng_close;
  }
#endif

  err = device_get_accessor_by_path(&ctx->ble, NULL, ble_name, DRIVER_CLASS_NET);
  if (err) {
    printk("Error while opening BLE network device: %d\n", err);
    goto crypto_close;
  }

  buffer_pool_init(&ctx->packet_pool, CONFIG_BLE_PACKET_SIZE,
                   packet_pool_grow, mem_scope_sys);

  ble_gattdb_init(&ctx->gattdb);

  err = net_scheduler_init(&ctx->scheduler, &ctx->packet_pool, rtc_name);
  if (err) {
    printk("Error while initializing net scheduler: %d\n", err);
    goto gatt_db_cleanup;
  }

#if defined(CONFIG_BLE_CRYPTO)
  err = ble_security_db_init(&ctx->security_db, persist_name, sec_name, &ctx->rng);
  if (err) {
    printk("Error while initializing peer db: %d\n", err);
    goto sched_cleanup;
  }
#endif

  return 0;

 sched_cleanup:
  net_scheduler_cleanup(&ctx->scheduler);
 gatt_db_cleanup:
  ble_gattdb_cleanup(&ctx->gattdb);
  buffer_pool_cleanup(&ctx->packet_pool);
 crypto_close:
#if defined(CONFIG_BLE_CRYPTO)
  device_put_accessor(&ctx->crypto);
  goto rng_close;
 rng_close2:
  dev_rng_cleanup(&rng);
 rng_close:
  dev_rng_cleanup(&ctx->rng);
#else
 rng_close:
#endif

  return err;
}

void ble_stack_context_cleanup(struct ble_stack_context_s *ctx)
{
  net_scheduler_cleanup(&ctx->scheduler);
  ble_gattdb_cleanup(&ctx->gattdb);
  buffer_pool_cleanup(&ctx->packet_pool);
#if defined(CONFIG_BLE_CRYPTO)
  dev_rng_cleanup(&ctx->rng);
  ble_security_db_cleanup(&ctx->security_db);
#endif
}

error_t ble_stack_context_address_non_resolvable_generate(struct ble_stack_context_s *ctx,
                                                          struct ble_addr_s *addr)
{
#if defined(CONFIG_BLE_CRYPTO)
  error_t err;

  err = dev_rng_wait_read(&ctx->rng, addr->addr, 6);
  if (err)
    return err;
#else
  for (size_t i = 0; i < 6; ++i)
    addr->addr[i] = random();
#endif
  
  ble_addr_random_type_set(addr, BLE_ADDR_RANDOM_NON_RESOLVABLE);

  return 0;
}

