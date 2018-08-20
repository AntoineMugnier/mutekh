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

#include "aes.h"

void soft_aes_ecb(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq)
{
  if (rq->len & 15)
    return;

  size_t i;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  for (i = 0; i < rq->len; i += 16)
    {
      uint32_t b[4];
      uint_fast8_t j;

      for (j = 0; j < 4; j++)
        b[j] = endian_be32_na_load(in + i + j * 4);

      if (rq->op & DEV_CRYPTO_INVERSE)
        soft_aes_decrypt(actx, b);
      else
        soft_aes_encrypt(actx, b);

      for (j = 0; j < 4; j++)
        endian_be32_na_store(out + i + j * 4, b[j]);
    }

  rq->error = 0;
}

