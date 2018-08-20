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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <string.h>
#include <mutek/printk.h>

#include <hexo/endian.h>
#include <ble/crypto.h>
#include <ble/protocol/address.h>

#include <device/class/crypto.h>

error_t ble_e(struct device_crypto_s *dev,
              const uint8_t *key,
              const uint8_t *in,
              uint8_t *out)
{
  struct dev_crypto_context_s ctx = {
    .mode = DEV_CRYPTO_MODE_ECB,
    .cache_ptr = NULL,
    .key_data = (void*)key,
    .key_len = 16,
  };

  struct dev_crypto_rq_s rq = {
    .op = 0,
    .ctx = &ctx,
    .in = in,
    .out = out,
    .len = 16,
  };

  return dev_crypto_wait_rq(dev, &rq);
}

error_t ble_ah(struct device_crypto_s *dev,
               const uint8_t *k,
               const uint8_t *r,
               uint8_t *out)
{
  uint8_t in[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, r[2], r[1], r[0]};
  uint8_t out_[16];
  uint8_t k_[16];
  error_t err;

  memrevcpy(k_, k, 16);

  err = ble_e(dev, k_, in, out_);
  out[0] = out_[15];
  out[1] = out_[14];
  out[2] = out_[13];

  return err;
}

error_t ble_c1(struct device_crypto_s *dev,
               const uint8_t *k,
               const uint8_t *r,
               const uint8_t *preq,
               const uint8_t *pres,
               const struct ble_addr_s *ia,
               const struct ble_addr_s *ra,
               uint8_t *out)
{
  uint8_t p[16];
  uint8_t in[16];
  uint8_t k_[16];

  memrevcpy(k_, k, 16);

  p[15] = ia->type;
  p[14] = ra->type;
  memrevcpy(p + 7, preq, 7);
  memrevcpy(p, pres, 7);

  memrevcpy(in, r, 16);
  memxor(in, in, p, 16);

  ble_e(dev, k_, in, out);

  memrevcpy(p + 10, ra->addr, 6);
  memrevcpy(p + 4, ia->addr, 6);
  memset(p, 0, 4);

  memxor(in, out, p, 16);

  ble_e(dev, k_, in, p);

  memrevcpy(out, p, 16);

  return 0;
}

error_t ble_s1(struct device_crypto_s *dev,
               const uint8_t *k,
               const uint8_t *r1,
               const uint8_t *r2,
               uint8_t *out)
{
  uint8_t in[16];
  uint8_t out_[16];
  uint8_t k_[16];

  memrevcpy(k_, k, 16);
  memrevcpy(in, r1, 8);
  memrevcpy(in + 8, r2, 8);

  ble_e(dev, k_, in, out_);

  memrevcpy(out, out_, 16);

  return 0;
}

error_t ble_aes_cmac(struct device_crypto_s *dev,
                     const uint8_t *k,
                     const uint8_t *msg, size_t len,
                     uint8_t *out)
{
  struct dev_crypto_context_s ctx = {
    .mode = DEV_CRYPTO_MODE_HMAC,
    .cache_ptr = NULL,
    .key_data = (void*)k,
    .key_len = 16,
  };

  struct dev_crypto_rq_s rq = {
    .op = DEV_CRYPTO_INIT | DEV_CRYPTO_INVERSE | DEV_CRYPTO_FINALIZE,
    .ctx = &ctx,
    .ad = msg,
    .ad_len = len,
    .out = out,
    .len = 16,
  };

  return dev_crypto_wait_rq(dev, &rq);
}

#if 0

error_t ble_sk_derive(const uint8_t *lstk, const uint8_t *skd, uint8_t *key)
{
    uint8_t k[16];
    uint8_t in[16];
    error_t err;

    ble_memcpy_r(k, lstk, 16);
    ble_memcpy_r(in, skd, 16);

    err = ble_e(k, in, key);

    return err;
}

error_t ble_s1(const uint8_t *k, const uint8_t *r1, const uint8_t *r2, uint8_t *out)
{
    // r1 and r2 are little endian,
    // r is big endian
    uint8_t r[16];
    uint8_t o[16];
    error_t err;

    // lsb of r2' (at r2[0]) becomes lsb of r (at r[15])
    // msb of r1' (at r1[7]) becomes msb of r (at r[0])
    ble_memcpy_r(r + 8, r2, 8);
    ble_memcpy_r(r, r1, 8);

    err = ble_e(k, r, o);

    ble_memcpy_r(out, o, 16);

    return err;
}

error_t ble_b1(const uint8_t *k, uint16_t div, uint16_t rand, uint8_t *out)
{
    // Store div and rand as big endian directly
    uint8_t in[16];
    uint8_t o[16];
    error_t err;

    memset(in, 0, 12);
    endian_be16_na_store(in + 12, rand);
    endian_be16_na_store(in + 14, div);

    err = ble_e(k, in, o);

    ble_memcpy_r(out, o, 16);

    return err;
}

/* error_t ccm_decrypt(const ccm_state *state, const uint8_t *in, uint8_t *out) */
/* { */

/*     return 0; */
/* } */

const struct ble_dh_key ble_dh_debug_key = {
  .pub = {
    .x = {
      0x20, 0xb0, 0x03, 0xd2, 0xf2, 0x97, 0xbe, 0x2c,
      0x5e, 0x2c, 0x83, 0xa7, 0xe9, 0xf9, 0xa5, 0xb9,
      0xef, 0xf4, 0x91, 0x11, 0xac, 0xf4, 0xfd, 0xdb,
      0xcc, 0x03, 0x01, 0x48, 0x0e, 0x35, 0x9d, 0xe6,
    },
    .y = {
      0xdc, 0x80, 0x9c, 0x49, 0x65, 0x2a, 0xeb, 0x6d,
      0x63, 0x32, 0x9a, 0xbf, 0x5a, 0x52, 0x15, 0x5c,
      0x76, 0x63, 0x45, 0xc2, 0x8f, 0xed, 0x30, 0x24,
      0x74, 0x1c, 0x8e, 0xd0, 0x15, 0x89, 0xd2, 0x8b,
    },
  },
  .priv = {
    .value = {
      0x3f, 0x49, 0xf6, 0xd4, 0xa3, 0xc5, 0x5f, 0x38,
      0x74, 0xc9, 0xb3, 0xe3, 0xd2, 0x10, 0x3f, 0x50,
      0x4a, 0xff, 0x60, 0x7b, 0xeb, 0x40, 0xb7, 0x99,
      0x58, 0x99, 0xb8, 0xa6, 0xcd, 0x3c, 0x1a, 0xbd,
    },
  },
};

#endif
