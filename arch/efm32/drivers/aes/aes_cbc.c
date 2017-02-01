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

void efm32_aes_cbc_encrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s *rq, const uint8_t *key)
{
  uint32_t ctrl = EFM32_AES_CTRL_XORSTART;
  EFM32_AES_KEY_CTRL(key, ctrl);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  size_t count = rq->len >> 4;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  uint8_t *iv = rq->iv_ctr;

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(iv + 12));
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(iv + 8));
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(iv + 4));
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(iv + 0));

  while (count--)
    {
      EFM32_AES_KEY_RELOAD(key);

      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_XORDATA_ADDR, endian_be32_na_load(in + 12));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_XORDATA_ADDR, endian_be32_na_load(in + 8));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_XORDATA_ADDR, endian_be32_na_load(in + 4));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_XORDATA_ADDR, endian_be32_na_load(in + 0));
      efm32_aes_wait();

      endian_be32_na_store(out + 12, cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 8,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 4,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 0,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      out += 16;
      in += 16;
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    memcpy(iv, out - 16, 16);

  rq->err = 0;
}

void efm32_aes_cbc_decrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s *rq, const uint8_t *key)
{
  uint32_t ctrl = EFM32_AES_CTRL_DATASTART | EFM32_AES_CTRL_DECRYPT(DECRYPTION);
  EFM32_AES_KEY_CTRL(key, ctrl);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  size_t count = rq->len >> 4;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  uint8_t *iv = rq->iv_ctr;

  uint32_t x0, x1, x2, x3;
  uint32_t y0, y1, y2, y3;

  x0 = endian_be32_na_load(iv + 12);
  x1 = endian_be32_na_load(iv + 8);
  x2 = endian_be32_na_load(iv + 4);
  x3 = endian_be32_na_load(iv + 0);

  while (count--)
    {
      EFM32_AES_KEY_RELOAD(key);

      y0 = endian_be32_na_load(in + 12);
      y1 = endian_be32_na_load(in + 8);
      y2 = endian_be32_na_load(in + 4);
      y3 = endian_be32_na_load(in + 0);

      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, y0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, y1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, y2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, y3);
      efm32_aes_wait();

      endian_be32_na_store(out + 12, x0 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 8,  x1 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 4,  x2 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 0,  x3 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

      x0 = y0;
      x1 = y1;
      x2 = y2;
      x3 = y3;

      out += 16;
      in += 16;
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      endian_be32_na_store(iv + 12, x0);
      endian_be32_na_store(iv + 8,  x1);
      endian_be32_na_store(iv + 4,  x2);
      endian_be32_na_store(iv + 0,  x3);
    }

  rq->err = 0;
}

