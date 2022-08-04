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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <device/class/crypto.h>
#include <hexo/enum.h>

#include <mutek/mem_alloc.h>

const char dev_crypto_mode_e[] = ENUM_DESC_DEV_CRYPTO_MODE_E;
const char dev_crypto_op_e[] = ENUM_DESC_DEV_CRYPTO_OP_E;

extern inline bool_t
dev_crypto_ctx_bind(struct dev_crypto_context_s *ctx,
                    struct dev_crypto_context_s *ctx_array[],
                    dev_crypto_context_id_t *next,
                    dev_crypto_context_id_t count);

extern inline uint8_t
dev_crypto_memcmp(const void *a, const void *b, size_t len);

error_t dev_rng_init(struct dev_rng_s *rng, const char *dev)
{
  error_t err;
  struct dev_crypto_info_s info;

  memset(rng, 0, sizeof(*rng));

  err = device_get_accessor_by_path(&rng->device.base, NULL, dev, DRIVER_CLASS_CRYPTO);
  if (err)
    return err;

  DEVICE_OP(&rng->device, info, &info);

  if (info.state_size) {
    rng->state_data = mem_alloc(info.state_size, mem_scope_sys);

    if (!rng->state_data) {
      err = -ENOMEM;
      device_put_accessor(&rng->device.base);
    }
  }

  return err;
}

void dev_rng_cleanup(struct dev_rng_s *rng)
{
  if (rng->state_data)
    mem_free(rng->state_data);
  device_put_accessor(&rng->device.base);
}

#ifdef CONFIG_MUTEK_CONTEXT_SCHED

extern inline error_t
dev_crypto_wait_rq(const struct device_crypto_s *accessor,
                   struct dev_crypto_rq_s *rq);

error_t dev_rng_wait_read(struct dev_rng_s *rng, void *data, size_t size)
{
  struct dev_crypto_context_s ctx = {
    .mode = DEV_CRYPTO_MODE_RANDOM,
    .state_data = rng->state_data,
  };
  struct dev_crypto_rq_s rq = {
    .ctx = &ctx,
    .op = DEV_CRYPTO_FINALIZE,
    .out = data,
    .len = size,
  };

  return dev_crypto_wait_rq(&rng->device, &rq);
}

error_t dev_rng_wait_seed(struct dev_rng_s *rng, const void *data, size_t size)
{
  struct dev_crypto_context_s ctx = {
    .mode = DEV_CRYPTO_MODE_RANDOM,
    .state_data = rng->state_data,
  };
  struct dev_crypto_rq_s rq = {
    .ctx = &ctx,
    .op = DEV_CRYPTO_INVERSE,
    .ad_len = size,
    .ad = data,
  };

  return dev_crypto_wait_rq(&rng->device, &rq);
}

error_t dev_rng_wait_seed_from_other(struct dev_rng_s *rng,
                                     struct dev_rng_s *other, size_t size)
{
  error_t err;
  uint8_t *tmp = alloca(size);

  err = dev_rng_wait_read(other, tmp, size);
  if (err)
    return err;

  return dev_rng_wait_seed(rng, tmp, size);
}
#endif

