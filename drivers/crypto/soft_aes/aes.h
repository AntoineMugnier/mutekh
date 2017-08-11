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

#ifndef _CRYPTO_AES_DRV_H_
#define _CRYPTO_AES_DRV_H_

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>

#if defined(CONFIG_DRIVER_CRYPTO_SOFT_AES256)
# define AES_ROUNDS 15
#elif defined(CONFIG_DRIVER_CRYPTO_SOFT_AES192)
# define AES_ROUNDS 13
#else
# define AES_ROUNDS 11
#endif

#define AES_OCB_L_COUNT                                                 \
  (1 + bit_msb_index(CONFIG_DRIVER_CRYPTO_SOFT_AES_OCB3_MAXBLOCKS + 1))

struct soft_aes_context_s
{
  union {
    uint32_t key[8];
    uint32_t rkey[AES_ROUNDS][4];
  };
  uint8_t rounds;

  union {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_OCB3
    struct {
      uint32_t          l32[AES_OCB_L_COUNT + 2][4];
      uint32_t          s32[6];
      uint8_t           nonce[16];
    }                   ocb;
#endif
  };
};

DRIVER_PV(struct soft_aes_private_s
{
  struct dev_request_dlqueue_s queue;

  struct soft_aes_context_s actx[CONFIG_DRIVER_CRYPTO_SOFT_AES_CTXCOUNT];
  struct dev_crypto_context_s *ctx[CONFIG_DRIVER_CRYPTO_SOFT_AES_CTXCOUNT];
  dev_crypto_context_id_t next;
});

struct soft_aes_state_s
{
  union {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_OCB3
    struct {
      uint32_t      offset[4];
      uint32_t      sum[4];
      size_t        i;
# ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_AD
      uint32_t      offset_a[4];
      uint32_t      sum_a[4];
      size_t        i_a;
# endif
    }               ocb;
#endif
  };
};

void soft_aes_expand_key(uint32_t *b, uint_fast8_t c, uint_fast8_t r);
void soft_aes_encrypt(const struct soft_aes_context_s *ctx, uint32_t buf[4]);
void soft_aes_decrypt(const struct soft_aes_context_s *ctx, uint32_t buf[4]);

void soft_aes_ecb(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq);
void soft_aes_cbc(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq);
void soft_aes_ctr(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq);
void soft_aes_ocb(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq, bool_t ctx_ok);
void soft_aes_cmac(struct soft_aes_context_s *actx,
                   struct dev_crypto_rq_s *rq);

#endif

