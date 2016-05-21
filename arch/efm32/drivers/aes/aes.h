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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2014

*/

#ifndef _EFM32_AES_DRV_H_
#define _EFM32_AES_DRV_H_

#include <mutek/kroutine.h>
#include <string.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>
#include <device/clock.h>

#include <arch/efm32/aes.h>

#define CONFIG_EFM32_AES_ADDR 0x400e0000

#define EFM32_AES_OCB_L_COUNT                                           \
  (8 * sizeof(int) - __builtin_clz(CONFIG_DRIVER_EFM32_AES_OCB3_MAXBLOCKS + 1))

struct efm32_aes_context_s
{
  /* cached decrypt key */
#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
  uint8_t                 dkey[32];
#else
  uint8_t                 dkey[16];
#endif

#ifdef CONFIG_DRIVER_EFM32_AES_OCB3
  union {
    uint8_t               ocb_l[EFM32_AES_OCB_L_COUNT + 2][16];
    uint32_t              ocb_l32[EFM32_AES_OCB_L_COUNT + 2][4];
  };
  uint8_t                 ocb_nonce[16];
  union {
    uint8_t               ocb_stretch[16 + 8];
    uint32_t              ocb_s32[6];
  };
#endif
};

struct efm32_aes_state_s
{
  union {
#ifdef CONFIG_DRIVER_EFM32_AES_RANDOM
    uint32_t rand32[8];
    uint8_t  rand8[32];
#endif
  };
};

DRIVER_PV(struct efm32_aes_private_s
{
  struct dev_request_dlqueue_s queue;

  struct dev_clock_sink_ep_s clk_ep;

  struct efm32_aes_context_s actx[CONFIG_DRIVER_EFM32_AES_CTXCOUNT];
  struct dev_crypto_context_s *ctx[CONFIG_DRIVER_CRYPTO_SOFT_AES_CTXCOUNT];
  dev_crypto_context_id_t next;
});

void efm32_aes_random(uint32_t state[8], uint32_t *out);

void efm32_aes_ocb_key(struct efm32_aes_context_s * __restrict__ actx,
                       const struct dev_crypto_context_s * __restrict__ ctx);

void efm32_aes_ocb_nonce(struct efm32_aes_context_s * __restrict__ actx,
                         const struct dev_crypto_rq_s * __restrict__ rq);

void efm32_aes_ocb(struct efm32_aes_context_s * __restrict__ actx,
                   struct dev_crypto_rq_s * __restrict__ rq,
                   const uint8_t * __restrict__ key);

void efm32_aes_ocb_decrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s * __restrict__ rq,
                           const uint8_t * __restrict__ key);

void efm32_aes_ocb_hash(struct efm32_aes_context_s * __restrict__ actx,
                        struct dev_crypto_rq_s * __restrict__ rq, const uint8_t *key,
                        uint32_t tag[4]);

void efm32_aes_ecb(struct efm32_aes_context_s * __restrict__ actx,
                   struct dev_crypto_rq_s *rq, const uint8_t *key);

void efm32_aes_cbc_encrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s *rq, const uint8_t *key);

void efm32_aes_cbc_decrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s *rq, const uint8_t *key);

void efm32_aes_ctr(struct efm32_aes_context_s * __restrict__ actx,
                   struct dev_crypto_rq_s *rq, const uint8_t *key);

#ifdef EFM32_AES_CTRL_KEYBUFEN
# ifdef CONFIG_DRIVER_EFM32_AES_KEY256 
#  define EFM32_AES_KEY_CTRL(key, ctrl)         \
  if (key != NULL)                              \
    ctrl |= EFM32_AES_CTRL_AES256;              \
  else                                          \
    ctrl |= EFM32_AES_CTRL_KEYBUFEN;
#  define EFM32_AES_KEY_RELOAD(key)             \
  if (key != NULL)                              \
    efm32_aes_load_key256(key);
# else
#  define EFM32_AES_KEY_CTRL(key, ctrl)         \
  ctrl |= EFM32_AES_CTRL_KEYBUFEN;
#  define EFM32_AES_KEY_RELOAD(key)
# endif
#else
# define EFM32_AES_KEY_CTRL(key, ctrl)
# define EFM32_AES_KEY_RELOAD(key)              \
  efm32_aes_load_key128l(key);
#endif

inline void efm32_aes_load_key128l(const uint8_t *key)
{
  uint_fast8_t i;
  for (i = 0; i < 16; i += 4)
    cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(0),
                     endian_be32_na_load(key + 12 - i));
}

inline void efm32_aes_get_key128(uint8_t *key)
{
  uint_fast8_t i;
  for (i = 0; i < 16; i += 4)
    endian_be32_na_store(key + 12 - i, cpu_mem_read_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(0)));
}

#ifdef EFM32_AES_CTRL_KEYBUFEN
inline void efm32_aes_load_key128h(const uint8_t *key)
{
  uint_fast8_t i;
  for (i = 0; i < 16; i += 4)
    cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_KEYH_ADDR(0),
                     endian_be32_na_load(key + 12 - i));
}
#endif

#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
inline void efm32_aes_load_key256(const uint8_t *key)
{
  uint_fast8_t i;
  for (i = 0; i < 32; i += 4)
    cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(i >> 2),
                     endian_be32_na_load(key + 28 - i));
}

inline void efm32_aes_get_key256(uint8_t *key)
{
  uint_fast8_t i;
  for (i = 0; i < 32; i += 4)
    endian_be32_na_store(key + 28 - i, cpu_mem_read_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(i >> 2)));
}
#endif

inline void efm32_aes_wait()
{
  while (cpu_mem_read_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_STATUS_ADDR) & EFM32_AES_STATUS_RUNNING)
    asm volatile("nop");
}

#endif

