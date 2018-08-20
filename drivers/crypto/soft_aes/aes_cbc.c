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

void soft_aes_cbc(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;
  uint_fast8_t i;
  size_t len = rq->len;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;

  if (rq->len & 15)
    return;
  if (ctx->iv_len != 16)
    return;

  uint32_t iv[4];
  for (i = 0; i < 4; i++)
    iv[i] = endian_be32_na_load(rq->iv_ctr + i * 4);

  if (rq->op & DEV_CRYPTO_INVERSE)
    {
      uint32_t t[4], b[4];
      for (; len >= 16; len -= 16)
	{
	  for (i = 0; i < 4; i++)
	    b[i] = t[i] = endian_be32_na_load(in + i * 4);
	  soft_aes_decrypt(actx, t);
	  for (i = 0; i < 4; i++)
	    {
	      endian_be32_na_store(out + i * 4, t[i] ^ iv[i]);
	      iv[i] = b[i];
	    }
	  in += 16;
          out += 16;
	}
    }
  else
    {
      for (; len >= 16; len -= 16)
	{
	  for (i = 0; i < 4; i++)
	    iv[i] ^= endian_be32_na_load(in + i * 4);
	  soft_aes_encrypt(actx, iv);
	  for (i = 0; i < 4; i++)
	    endian_be32_na_store(out + i * 4, iv[i]);
	  in += 16;
          out += 16;
	}
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    for (i = 0; i < 4; i++)
      endian_be32_na_store(rq->iv_ctr + i * 4, iv[i]);

  rq->error = 0;
}

