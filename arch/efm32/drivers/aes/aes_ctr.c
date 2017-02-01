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

#include "aes.h"

void efm32_aes_ctr(struct efm32_aes_context_s * __restrict__ actx,
                   struct dev_crypto_rq_s *rq, const uint8_t *key)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;

  uint32_t ctrl = EFM32_AES_CTRL_DATASTART;
  EFM32_AES_KEY_CTRL(key, ctrl);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  size_t l = rq->len;
  uint32_t x0, x1, x2, x3;

  union {
    uint8_t  p8[16];
    uint32_t p32[4];
  }          p;

  memcpy(p.p8 + 16 - ctx->iv_len, rq->iv_ctr, ctx->iv_len);
  memset(p.p8, 0, 16 - ctx->iv_len);
  x0 = endian_be32(p.p32[3]);
  x1 = endian_be32(p.p32[2]);
  x2 = endian_be32(p.p32[1]);
  x3 = endian_be32(p.p32[0]);

  while (1)
    {
      EFM32_AES_KEY_RELOAD(key);

      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, x0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, x1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, x2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, x3);
      efm32_aes_wait();

      p.p32[3] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[2] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[1] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[0] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

      uint_fast8_t i;
      if (l < 16)
        {
          for (i = 0; i < l; i++)
            out[i] = in[i] ^ p.p8[i];
          break;
        }

      for (i = 0; i < 16; i++)
        out[i] = in[i] ^ p.p8[i];

      uint64_t t;
      x0 = t = (uint64_t)x0 + 1;
      x1 = t = (uint64_t)x1 + (t >> 32);
      x2 = t = (uint64_t)x2 + (t >> 32);
      x3 = t = (uint64_t)x3 + (t >> 32);

      out += 16;
      in += 16;
      l -= 16;
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      p.p32[3] = endian_be32(x0);
      p.p32[2] = endian_be32(x1);
      p.p32[1] = endian_be32(x2);
      p.p32[0] = endian_be32(x3);
      memcpy(rq->iv_ctr, p.p8 + 16 - ctx->iv_len, ctx->iv_len);
    }

  rq->err = 0;
}

