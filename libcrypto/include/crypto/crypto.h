/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef _CRYPTO_H_
#define _CRYPTO_H_

#include <hexo/types.h>

/***********************************************************************
 *    Hash Algorithms
 */

#define CRYPTO_HASH_INIT(n) void (n)(void *ctx_)

/** @this reinitializes the hash context */
typedef CRYPTO_HASH_INIT(crypto_hash_init_t);

#define CRYPTO_HASH_UPDATE(n) void (n)(void *ctx_, const uint8_t *data, size_t len)

/** @this updates the hash context with passed data */
typedef CRYPTO_HASH_UPDATE(crypto_hash_update_t);

#define CRYPTO_HASH_GET(n) void (n) (void *ctx_, uint8_t *hash)

/** @this get the hash result */
typedef CRYPTO_HASH_GET(crypto_hash_get_t);

/** @this describes a hash algorithm */
struct crypto_hash_algo_s
{
  crypto_hash_init_t *f_init;
  crypto_hash_update_t *f_update;
  crypto_hash_get_t *f_get;
  size_t ctx_size;		//< size of context struct for this algorithm
  size_t hash_size;		//< size of the hash result
};

/***********************************************************************
 *    Pseudo Random Generator Algorithms
 */

#define CRYPTO_STREAM_INIT(n) void (n)(void *ctx_)

/** @this reinitializes the stream generator */
typedef CRYPTO_STREAM_INIT(crypto_stream_init_t);

#define CRYPTO_STREAM_UPDATE(n) void (n)(void *ctx_, const uint8_t *key, size_t keylen)

/** @this updates the stream generator state using given key */
typedef CRYPTO_STREAM_UPDATE(crypto_stream_update_t);

#define CRYPTO_STREAM_GETSTREAM(n) void (n)(void *ctx_, uint8_t *data, size_t data_len)

/** @this gets pseudo random stream. The @tt data parameter
    may be @ref NULL to update generator state but discard output stream. */
typedef CRYPTO_STREAM_GETSTREAM(crypto_stream_getstream_t);

#define CRYPTO_STREAM_XORSTREAM(n) void (n)(void *ctx_, uint8_t *data, size_t data_len)

/** @this xor a buffer with pseudo random stream. */
typedef CRYPTO_STREAM_XORSTREAM(crypto_stream_xorstream_t);

struct crypto_stream_algo_s
{
  crypto_stream_init_t *f_init;
  crypto_stream_update_t *f_update;
  crypto_stream_getstream_t *f_getstream;
  crypto_stream_xorstream_t *f_xorstream;
  size_t ctx_size;              //< size of context struct for this algorithm
  size_t min_key;		//< minimal size of data for the update function
  size_t max_key;		//< maximal size of data for the update function
};

#endif

