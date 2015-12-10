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

/**
 * @file
 * @module{Devices support library}
 * @short Cipher device driver API
 */

#ifndef __DEVICE_CRYPTO_H__
#define __DEVICE_CRYPTO_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>

#include <device/driver.h>
#include <device/request.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

struct device_s;
struct driver_s;
struct device_crypto_s;
struct driver_crypto_s;

ENUM_DESCRIPTOR(dev_crypto_capabilities_e, strip:DEV_CRYPTO_CAP_, upper, or);

enum dev_crypto_capabilities_e
{
  /** Algorithm can work with 64 bits keys. */
  DEV_CRYPTO_CAP_64BITS_KEY    = 0x0010,
  /** Algorithm can work with 128 bits keys. */
  DEV_CRYPTO_CAP_128BITS_KEY   = 0x0020,
  /** Algorithm can work with 192 bits keys. */
  DEV_CRYPTO_CAP_192BITS_KEY   = 0x0040,
  /** Algorithm can work with 256 bits keys. */
  DEV_CRYPTO_CAP_256BITS_KEY   = 0x0080,
  /** Algorithm can work with any key length. */
  DEV_CRYPTO_CAP_VARLEN_KEY    = 0x0100,
  /** Implementation is able to write output data in the same buffer
      as input data. */
  DEV_CRYPTO_CAP_INPLACE       = 0x0200,
  /** Implementation is able to write output data in a different
      buffer from input data. */
  DEV_CRYPTO_CAP_NOTINPLACE    = 0x0400,
  /** Implementations is able to handle requests where @ref
      DEV_CRYPTO_INIT and @ref DEV_CRYPTO_FINALIZE are not both
      set. This is relevant for mode of operations which can be
      stateful. @see dev_crypto_context_s::state_data */
  DEV_CRYPTO_CAP_STATEFUL      = 0x0800,
  /** The authentication operations support associated data. */
  DEV_CRYPTO_CAP_ASSOC_DATA    = 0x1000,
};

struct dev_crypto_info_s
{
  /** Name of the algorithm */
  const char                    *name;

  /** Mask of supported modes of operation.
      @see dev_crypto_mode_e */
  uint16_t                      modes_mask;

  /** Byte size of the crypto state data.
      @see dev_crypto_context_s::state_data */
  uint16_t                      state_size;

  /** required alignment of data buffers */
  uint8_t                       align_log2;

  /** capability flags */
  enum dev_crypto_capabilities_e BITFIELD(cap,16);

  /** algorithm block size in bytes, relevant for block ciphers and
      hash based algorithms. */
  uint8_t                       block_len;
};

ENUM_DESCRIPTOR(dev_crypto_mode_e, strip:DEV_CRYPTO_MODE_, upper);

enum dev_crypto_mode_e
{
  /** Electronic codebook cipher block mode. When this mode is used,
      the @tt in, @tt out and @tt len fields of the request are
      used. The data and iv lengths must be multiple of the cipher
      block size. */
  DEV_CRYPTO_MODE_ECB,
  /** Cipher-block chaining mode. When this mode is used, the @tt in,
      @tt out, @tt len and @tt iv_ctr fields of the request are
      used. The @tt len and @tt iv_len values must be multiple of the
      cipher block size. The passed @tt iv_ctr buffer is updated if
      @tt DEV_CRYPTO_FINALIZE is set. */
  DEV_CRYPTO_MODE_CBC,
  /** Cipher Feedback mode. Usage is similar to
      @ref DEV_CRYPTO_MODE_CBC. */
  DEV_CRYPTO_MODE_CFB,
  /** Output Feedback cipher mode. Usage is similar to
      @ref DEV_CRYPTO_MODE_CBC. */
  DEV_CRYPTO_MODE_OFB,

  /** Counter cipher mode. When this mode is used, the @tt in, @tt
      out, @tt len and @tt iv_ctr fields of the request are used. The
      value of @tt iv_len can be less than the cipher block size. In
      this case the msb of the counter padded with 0.

      The value of @tt len may not be a multiple of the cipher block
      size. The counter is not updated after processing of a partial
      block.

      The passed @tt iv_ctr buffer is updated if @tt
      DEV_CRYPTO_FINALIZE is set.
  */
  DEV_CRYPTO_MODE_CTR,

  /** Galois Counter cipher Mode. When this mode is used, the @tt in,
      @tt out, @tt len, @tt iv_ctr, @tt ad_len and @tt auth fields of
      the request are used. If the @tt ad_len field is not 0, the @tt
      ad field must be a valid pointer to the associated data.

      The @ref DEV_CRYPTO_INIT and @ref DEV_CRYPTO_FINALIZE operation
      flags are relevant and a state buffer must be provided when they
      are not both set. The @tt auth field must be valid when @tt
      DEV_CRYPTO_FINALIZE is set. The authentication tag is either
      stored in the @tt auth buffer or checked against it depending on
      the @ref DEV_CRYPTO_INVERSE flag.

      The @tt len and @tt ad_len values must be multiple of the cipher
      block size when @ref DEV_CRYPTO_FINALIZE is not set. */
  DEV_CRYPTO_MODE_GCM,
  /** Usage is similar to @ref DEV_CRYPTO_MODE_GCM. */
  DEV_CRYPTO_MODE_CCM,
  /** OCB3 cipher mode as specified in rfc7253. Usage is similar to
      @ref DEV_CRYPTO_MODE_GCM. The @tt iv_ctr field points to the
      nonce. */
  DEV_CRYPTO_MODE_OCB3,

  /** Stream cipher mode. When this mode is used, the @tt in, @tt out,
      @tt len and @tt iv_ctr fields or the request are used. The input
      data is xored with the generated key stream and stored to the
      output buffer. If the @tt out field is @tt NULL, the generated
      key stream is discarded. If the @tt in field is @tt NULL, the
      key stream is stored instead of being xored. The cipher is
      initialized when the @ref DEV_CRYPTO_INIT operation flag is set,
      taking the IV into account when relevant. The @ref
      DEV_CRYPTO_FINALIZE flag can be used when a state buffer is not
      provided. */
  DEV_CRYPTO_MODE_STREAM,

  /** Hash processing mode. When this mode is used, the @tt ad, @tt
      ad_len, @tt out and @tt len fields are used.  No key is
      required. The @ref DEV_CRYPTO_INIT and @ref DEV_CRYPTO_FINALIZE
      operation flags are relevant and a state buffer must be provided
      when they are not both set. The input data is pointed to by @tt
      ad. The @tt out buffer must be valid when @tt
      DEV_CRYPTO_FINALIZE is set. Variable length output hash
      algorithms with a stateful implementation may support extra
      finalizing requests with a zero length input. */
  DEV_CRYPTO_MODE_HASH,

  /** Hash based message authentication code. When this mode is used,
      the @tt in, @tt len and @tt auth fields are used. The @ref
      DEV_CRYPTO_INIT and @ref DEV_CRYPTO_FINALIZE operation flags are
      relevant and a state buffer must be provided when they are not
      both set. The @tt auth field must be valid when @tt
      DEV_CRYPTO_FINALIZE. The authentication tag is either stored in
      the @tt auth buffer or checked against it depending on the @ref
      DEV_CRYPTO_INVERSE flag. */
  DEV_CRYPTO_MODE_HMAC,

  /** Random data generator mode. When the @ref DEV_CRYPTO_INVERSE
      operation flag is set, the random generator is seeded with data
      from the buffer specified by the @tt ad and @tt ad_len fields of
      the request. When the @ref DEV_CRYPTO_FINALIZE operation flag is
      set, random data is stored in the buffer specified by the @tt
      out and @tt len fields. A state buffer must always be provided
      and the @ref DEV_CRYPTO_INIT operation flag must be used to
      initialize the state on the initial request. No key is used. */
  DEV_CRYPTO_MODE_RANDOM,
};

struct dev_crypto_context_s
{
  /** Pointer to the device which last used this key. May be set by
      the driver to reuse the internally expanded key. This field must
      be set to @tt NULL when the value of key_data, key_len, mode,
      iv_len or data field is updated. */
  void                          *cache_ptr;

  /** Algorithm internal state. Used to store the state when either
      @ref DEV_CRYPTO_INIT or @ref DEV_CRYPTO_FINALIZE are not set in
      the request.
      @see dev_crypto_info_s:state_size @see DEV_CRYPTO_CAP_STATEFUL */
  void                          *state_data;

  /** Key data */
  uint8_t                       *key_data;
  /** Key length in bytes. */
  uint8_t                       key_len;

  /** Context id internal to the driver @see dev. */
  uint8_t                       cache_id;

  /** Byte length of the IV/counter/nonce buffer. */
  uint8_t                       iv_len;
  /** Byte length of the authentication tag. If the length is 0, the
      tag is not generated/checked. */
  uint8_t                       auth_len;

  /** Used mode of operation */
  enum dev_crypto_mode_e        BITFIELD(mode,8);

  /** This hint flag can be set if the context will not be used
      several times. This may prevent the driver from using an
      internal state cache entry. */
  bool_t                        BITFIELD(ephemeral,1);
  /** This hint flag can be set if the context will only be used to
      perform encryption. The driver may skip expanding the key for
      this purpose. */
  bool_t                        BITFIELD(encrypt_only,1);
  /** This hint flag can be set if the context will only be used to
      perform decryption. */
  bool_t                        BITFIELD(decrypt_only,1);
};

ENUM_DESCRIPTOR(dev_crypto_op_e, strip:DEV_CRYPTO_, upper, or, empty);

enum dev_crypto_op_e
{
  /** Must be defined when the first chunk of data is processeced by
      either a cipher with authentication or a hash algorithm. */
  DEV_CRYPTO_INIT     = 1,
  /** Inverse the processing direction. For ciphers, this means
      performing data decryption. */
  DEV_CRYPTO_INVERSE  = 2,
  /** Must be defined when the last chunk of data is processeced by
      either a cipher with authentication or a hash algorithm. */
  DEV_CRYPTO_FINALIZE = 4,
};

struct dev_crypto_rq_s
{
  struct dev_request_s                  rq;

  enum dev_crypto_op_e                  BITFIELD(op,3);

  error_t                               err;

  /** Key used for encryption/decryption */
  struct dev_crypto_context_s           *ctx;

  /** Size of data in the input and output buffers. */
  size_t                                len;
  /** Input data. */
  const uint8_t                         *in;
  /** Output data, may be the same as @tt in. */
  uint8_t                               *out;

  /** Size of data in the @tt ad buffer. */
  size_t                                ad_len;
  /** Associated data and hash input data. */
  const uint8_t                         *ad;

  /** Pointer to IV/counter/nonce data relevant to various modes. When
      the size of the buffer is not fixed for the mode in use, it is
      defined by @ref dev_crypto_context_s:iv_len. */
  uint8_t                               *iv_ctr;

  /** Pointer to authentication data. The size of the buffer is
      defined by @ref dev_crypto_context_s:auth_len. */
  uint8_t                               *auth;
};

STRUCT_INHERIT(dev_crypto_rq_s, dev_request_s, rq);

#define DEVCRYPTO_INFO(n)	error_t  (n) (struct device_crypto_s *accessor, \
                                              struct dev_crypto_info_s *info)
typedef DEVCRYPTO_INFO(devcrypto_info_t);

#define DEVCRYPTO_REQUEST(n) void  (n) (struct device_crypto_s *accessor,   \
                                        struct dev_crypto_rq_s *rq)
typedef DEVCRYPTO_REQUEST(devcrypto_request_t);

DRIVER_CLASS_TYPES(crypto,
                   devcrypto_info_t *f_info;
                   devcrypto_request_t *f_request;
                   );

#define DRIVER_CRYPTO_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_crypto_s){   \
    .class_ = DRIVER_CLASS_CRYPTO,                                  \
    .f_info = prefix ## _info,                                      \
    .f_request = prefix ## _request,                                \
  })

config_depend(CONFIG_DEVICE_CRYPTO)
inline error_t
dev_crypto_spin_op(struct device_crypto_s *accessor,
                   struct dev_crypto_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_spin_init(&rq->rq, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_spin_wait(&st);
  return rq->err;
}

/** Synchronous memory device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2(CONFIG_DEVICE_CRYPTO, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_crypto_wait_op(struct device_crypto_s *accessor,
                   struct dev_crypto_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_sched_init(&rq->rq, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_sched_wait(&st);
  return rq->err;
}

typedef uint8_t dev_crypto_context_id_t;

/** @internal @This helper function can be used by the crypto driver
    to handle binding of its internally cached crypto context data to
    the @ref dev_crypto_context_s of a request. An internal context
    entry is reused if an existing association is still valid. The @tt
    ctx->cache_id field is updated to the index of the retained
    internal context entry.

    @param ctx crypto context of the request
    @param ctx_array array of pointers to cached contexts in
           driver private data.
    @param next pointer to next cache victim entry in
           driver private data.
    @param count total number of entries in the cache.

    This function returns true if the retained entry still contains
    valid data. The driver must perform the key schedule and recompute
    other context related data again if this function returns false.
*/
inline bool_t
dev_crypto_ctx_bind(struct dev_crypto_context_s *ctx,
                    struct dev_crypto_context_s *ctx_array[],
                    dev_crypto_context_id_t *next,
                    dev_crypto_context_id_t count)
{
  bool_t ctx_ok = (ctx->cache_ptr == ctx_array);

  if (ctx_ok)
    {
      if (ctx->cache_id >= count)
        ctx_ok = 0;
      else
        {
          if (ctx_array[ctx->cache_id] != ctx)
            ctx_ok = 0;
        }
    }

  if (!ctx_ok)
    {
      dev_crypto_context_id_t n = *next;
      ctx->cache_id = n;
      if (!ctx->ephemeral)
        {
          if (++n >= count)
            n = 0;
          *next = n;
        }
      ctx_array[ctx->cache_id] = ctx;
      ctx->cache_ptr = ctx_array;
    }

  return ctx_ok;
}

inline uint8_t dev_crypto_memcmp(const void *a, const void *b, size_t len)
{
  const uint8_t *a_ = a, *b_ = b;
  uint8_t c = 0;
  while (len--)
    c |= *a_++ ^ *b_++;
  return c;
}

/**
   A random-number generator context, contains a handle on a device
   and internal state for a RNG.
 */
struct dev_rng_s
{
  struct device_crypto_s device;
  void *state_data;
};

/**
   @this initializes a random number generator context.

   @param rng RNG backing device to open
   @param dev Device path to open
   @returns 0 on success, or an error
 */
config_depend(CONFIG_DEVICE_CRYPTO)
error_t dev_rng_init(struct dev_rng_s *rng, const char *dev);

/**
   @this releases all objects associated to RNG context

   @param rng RNG context to close
 */
config_depend(CONFIG_DEVICE_CRYPTO)
void dev_rng_cleanup(struct dev_rng_s *rng);

/**
   @this retrieves a random data stream from device

   @param rng RNG context to read from
   @param data Data pointer to write to
   @param size Byte size to read
   @returns 0 on success, or an error
 */
config_depend_and2(CONFIG_DEVICE_CRYPTO, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rng_wait_read(struct dev_rng_s *rng, void *data, size_t size);

/**
   @this seeds the RNG with some data

   @param rng RNG context to seed
   @param data Data pointer to read from
   @param size Byte size to read
   @returns 0 on success, or an error
 */
config_depend_and2(CONFIG_DEVICE_CRYPTO, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rng_wait_seed(struct dev_rng_s *rng, const void *data, size_t size);

/**
   @this extracts a random stream from a RNG and uses it as seed for
   another one.

   @param rng RNG context to seed
   @param other RNG context to read data from
   @param size Byte size to pass from one to the other
   @returns 0 on success, or an error
 */
config_depend_and2(CONFIG_DEVICE_CRYPTO, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rng_wait_seed_from_other(struct dev_rng_s *rng,
                                     struct dev_rng_s *other, size_t size);

#ifdef CONFIG_DEVICE_CRYPTO
# define DEV_STATIC_RES_DEV_CRYPTO(path_)                               \
  DEV_STATIC_RES_DEVCLASS_PARAM("crypto", path_, DRIVER_CLASS_CRYPTO)
#else
# define DEV_STATIC_RES_DEV_CRYPTO(path_)                               \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif
