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

#define LOGK_MODULE_ID "lble"

#include <ble/stack/context.h>
#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>
#include <ble/protocol/l2cap.h>

#include <ble/crypto.h>

#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>

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
                               struct persist_context_s *persist)
{
  error_t err;
  struct dev_rng_s rng;

  err = dev_rng_init(&ctx->rng, sec_name);
  if (err) {
    logk_error("Error while initing DRBG: %d", err);
    return err;
  }

  err = dev_rng_init(&rng, rng_name);
  if (err) {
    logk_error("Error while initing RNG: %d", err);
    goto rng_close;
  }

  err = dev_rng_wait_seed_from_other(&ctx->rng, &rng, 16);
  if (err) {
    logk_error("Error while seeding from RNG: %d", err);
    goto rng_close2;
  }

  dev_rng_cleanup(&rng);

#if defined(CONFIG_BLE_CRYPTO)
  err = device_get_accessor_by_path(&ctx->crypto.base, NULL, sec_name, DRIVER_CLASS_CRYPTO);
  if (err) {
    logk_error("Error while opening crypto device: %d", err);
    goto rng_close;
  }
#endif

  err = device_get_accessor_by_path(&ctx->ble.base, NULL, ble_name, DRIVER_CLASS_NET);
  if (err) {
    logk_error("Error while opening BLE network device: %d", err);
    goto crypto_close;
  }

  buffer_pool_init(&ctx->packet_pool, CONFIG_BLE_PACKET_SIZE,
                   packet_pool_grow, mem_scope_sys);

  ble_gattdb_init(&ctx->gattdb);

  err = net_scheduler_init(&ctx->scheduler, &ctx->packet_pool, rtc_name);
  if (err) {
    logk_error("Error while initializing net scheduler: %d", err);
    goto gatt_db_cleanup;
  }

#if defined(CONFIG_BLE_CRYPTO)
  err = ble_security_db_init(&ctx->security_db, persist, sec_name, &ctx->rng);
  if (err) {
    logk_error("Error while initializing peer db: %d", err);
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
  device_put_accessor(&ctx->crypto.base);
  goto rng_close;
#endif
 rng_close2:
  dev_rng_cleanup(&rng);
 rng_close:
  dev_rng_cleanup(&ctx->rng);

  return err;
}

void ble_stack_context_cleanup(struct ble_stack_context_s *ctx)
{
  net_scheduler_cleanup(&ctx->scheduler);
  ble_gattdb_cleanup(&ctx->gattdb);
  buffer_pool_cleanup(&ctx->packet_pool);
#if defined(CONFIG_BLE_CRYPTO)
  ble_security_db_cleanup(&ctx->security_db);
#endif
  dev_rng_cleanup(&ctx->rng);
}

error_t ble_stack_context_address_non_resolvable_generate(struct ble_stack_context_s *ctx,
                                                          struct ble_addr_s *addr)
{
  error_t err;

  err = dev_rng_wait_read(&ctx->rng, addr->addr, 6);
  if (err) {
    logk_error("RNG failure: %d", err);
    return err;
  }

  ble_addr_random_type_set(addr, BLE_ADDR_RANDOM_NON_RESOLVABLE);

  return 0;
}

#if defined(CONFIG_BLE_SECURITY_DB)
error_t ble_stack_context_address_resolvable_generate(struct ble_stack_context_s *ctx,
                                                      struct ble_addr_s *addr)
{
  error_t err;

  err = dev_rng_wait_read(&ctx->rng, addr->addr + 3, 3);
  if (err) {
    logk_error("RNG failure: %d", err);
    return err;
  }

  ble_addr_random_type_set(addr, BLE_ADDR_RANDOM_RESOLVABLE);

  return ble_ah(&ctx->crypto, ctx->security_db.irk, addr->addr + 3, addr->addr);
}
#endif

error_t ble_stack_context_local_address_get(struct ble_stack_context_s *ctx,
                                            struct ble_addr_s *addr)
{
  error_t err;
  struct dev_net_info_s info;

  err = DEVICE_OP(&ctx->ble, get_info, &info);
  if (err)
    return err;

  ble_addr_net_parse(addr, &info.addr);

  return 0;
}

uint32_t ble_stack_access_address_generate(struct ble_stack_context_s *ctx)
{
  error_t err;

  for (;;) {
    uint8_t tmp[16];

    err = dev_rng_wait_read(&ctx->rng, tmp, 16);
    if (err) {
      logk_error("RNG failure: %d", err);
      return -1;
    }

    for (uint8_t offset = 0; offset <= 12; ++offset) {
      uint32_t aa = endian_le32_na_load(tmp + offset);

      if (ble_data_aa_is_valid(aa))
        return aa;
    }
  }
}

static void adv_data_append(uint8_t **buffer, size_t *buffer_size,
                            const uint8_t type,
                            const void *data, const uint8_t size)
{
  if (*buffer_size < size + 2 || size == 0)
    return;

  uint8_t *ptr = *buffer;

  *ptr++ = size + 1;
  *ptr++ = type;
  memcpy(ptr, data, size);

  *buffer += size + 2;
  *buffer_size -= size + 2;
}

void ble_stack_context_ad_collect(struct ble_stack_context_s *context,
                            uint8_t *ad_, size_t ad_size_max,
                            size_t *ad_size_used)
{
  static const size_t srv_max_count = 4;
  uint16_t srv16_list[srv_max_count];
  uint8_t srv128_list[16];
  const void *data;
  size_t size, ad_left, value_size;
  uint8_t *ad = ad_;

  ad_left = ad_size_max;

  uint8_t flags = BLE_GAP_FLAGS_BREDR_NOT_SUPPORTED;
  flags |= BLE_GAP_FLAGS_LIMITED_ADV;
  adv_data_append(&ad, &ad_left, BLE_GAP_FLAGS, &flags, sizeof(flags));

  if (!ble_gattdb_std_char_read(&context->gattdb,
                                 BLE_GATT_SERVICE_GENERIC_ACCESS,
                                 BLE_GATT_CHAR_GAP_APPEARANCE,
                                 &data, &size))
    adv_data_append(&ad, &ad_left, BLE_GAP_APPEARANCE, data, size);

  value_size = ble_gattdb_srv16_list_get(&context->gattdb, srv16_list, sizeof(srv16_list));
  if (value_size)
    adv_data_append(&ad, &ad_left,
                    value_size <= srv_max_count ? BLE_GAP_UUID16_SERVICE_LIST_COMPLETE
                    : BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE,
                    srv16_list, __MIN(sizeof(srv16_list), value_size));

  value_size = ble_gattdb_srv128_list_get(&context->gattdb, srv128_list, sizeof(srv128_list));
  if (value_size)
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_UUID128_SERVICE_LIST_COMPLETE,
                    srv128_list, __MIN(sizeof(srv128_list), value_size));

  if (!ble_gattdb_std_char_read(&context->gattdb,
                                 BLE_GATT_SERVICE_GENERIC_ACCESS,
                                 BLE_GATT_CHAR_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
                                 &data, &size))
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_SLAVE_CONNECTION_INTERVAL_RANGE, data, 4);

  if (!ble_gattdb_std_char_read(&context->gattdb,
                                 BLE_GATT_SERVICE_GENERIC_ACCESS,
                                 BLE_GATT_CHAR_GAP_DEVICE_NAME,
                                 &data, &size))
    adv_data_append(&ad, &ad_left,
                    BLE_GAP_COMPLETE_LOCAL_NAME, data, size);

  *ad_size_used = ad_size_max - ad_left;
}

void ble_stack_context_use(struct ble_stack_context_s *ctx)
{
  device_start(&ctx->ble.base);
}

void ble_stack_context_release(struct ble_stack_context_s *ctx)
{
  device_stop(&ctx->ble.base);
}
