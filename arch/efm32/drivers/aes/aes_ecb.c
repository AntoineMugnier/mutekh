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

void efm32_aes_ecb(struct efm32_aes_context_s * __restrict__ actx,
                   struct dev_crypto_rq_s *rq, const uint8_t *key)
{
  uint32_t ctrl = EFM32_AES_CTRL_DATASTART;
  if (rq->op & DEV_CRYPTO_INVERSE)
    ctrl |= EFM32_AES_CTRL_DECRYPT(DECRYPTION);
  EFM32_AES_KEY_CTRL(key, ctrl);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  if (rq->len & 15)
    return;

  size_t count = rq->len >> 4;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;

  while (count--)
    {
      EFM32_AES_KEY_RELOAD(key);

      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 12));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 8));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 4));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 0));
      efm32_aes_wait();

      endian_be32_na_store(out + 12, cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 8,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 4,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 0,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      out += 16;
      in += 16;
    }

  rq->error = 0;
}
