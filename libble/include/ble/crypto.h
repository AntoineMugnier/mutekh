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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_CRYPTO_H_
#define BLE_CRYPTO_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Bluetooth cryptography toolbox

   Those functions are from "Cryptographic toolbox" chapter of
   Bluetooth Core Specification.  See Core_v4.2, Vol. 3, Part. H.
*/

#include <hexo/error.h>

struct ble_addr_s;
struct buffer_s;

/**
   @this defines a public key object
 */
struct ble_dh_public_key {
  uint8_t x[32];
  uint8_t y[32];
};

/**
   @this defines a private key object
 */
struct ble_dh_private_key {
  uint8_t value[32];
};

/**
   @this defines a key pair object
 */
struct ble_dh_key {
  struct ble_dh_public_key pub;
  struct ble_dh_private_key priv;
};

struct device_crypto_s;
struct dev_crypto_rq_s;

/**
   Cryptographic toolbox @tt e function (2.2.1)
   Actually, a AES encrypt.

   Key, in and out are in usual ECB order (what Bluetooth spec
   explains as big-endian).
 */
error_t ble_e(const struct device_crypto_s *dev,
              const uint8_t *key,
              const uint8_t *in,
              uint8_t *out);

/**
   @this is a BLE-customized AES-CCM encrypt.  Expected @tt clear and
   @tt ciphered buffers contain link-layer header.

   @tt iv is in Bluetooth network order.
 */
error_t ble_ccm_encrypt(
  struct device_crypto_s *dev,
  const uint8_t *k,
  const uint8_t *iv,
  uint32_t packet_count, uint8_t way,
  const struct buffer_s *clear,
  struct buffer_s *ciphered);

/**
   @this is a BLE-customized AES-CCM decrypt.  Expected @tt clear and
   @tt ciphered buffers contain link-layer header.

   @tt iv is in Bluetooth network order.
 */
error_t ble_ccm_decrypt(
  struct device_crypto_s *dev,
  const uint8_t *k,
  const uint8_t *iv,
  uint32_t packet_count, uint8_t way,
  const struct buffer_s *ciphered,
  struct buffer_s *clear);

/**
   Cryptographic toolbox @tt ah function (2.2.2)

   @tt k is in Bluetooth order.
   @tt r and @tt out both are 3-byte long.

   This function is usually used for random address resolution with
   @tt IRK.
 */
error_t ble_ah(const struct device_crypto_s *dev,
               const uint8_t *k,
               const uint8_t *r,
               uint8_t *out);

/**
   Cryptographic toolbox @tt c1 function (2.2.3)

   This function is usually used for (legacy) pairing confirmation.
 */
error_t ble_c1(const struct device_crypto_s *dev,
               const uint8_t *k,
               const uint8_t *r,
               const uint8_t *preq,
               const uint8_t *pres,
               const struct ble_addr_s *ia,
               const struct ble_addr_s *ra,
               uint8_t *out);

/**
   Cryptographic toolbox @tt s1 function (2.2.4)

   @tt Key, @tt r1, @tt r2 and out are in Bluetooth (little-endian)
   order.
 */
error_t ble_s1(const struct device_crypto_s *dev,
               const uint8_t *k,
               const uint8_t *r1,
               const uint8_t *r2,
               uint8_t *out);

/**
   Cryptographic toolbox @tt AES-CMAC function (2.2.5)

   @tt Key and @tt out are in bluetooth (little-endian) order.

   @tt data and @tt size point a byte-stream.
 */
error_t ble_aes_cmac(const struct device_crypto_s *dev,
                     const uint8_t *k,
                     const uint8_t *data,
                     size_t size,
                     uint8_t *out);

/**
   Cryptographic toolbox @tt f4 function (2.2.6)

   This function is usually used for (secure) pairing confirmation.
 */
error_t ble_f4(const struct device_crypto_s *dev,
               const uint8_t *u,
               const uint8_t *v,
               const uint8_t *x,
               uint8_t z,
               uint8_t *out);

/**
   Cryptographic toolbox @tt f5 function (2.2.7)

   This function is usually used for key generation.
 */
error_t ble_f5(const struct device_crypto_s *dev,
               const uint8_t *w,
               const uint8_t *n1, const uint8_t *n2,
               const struct ble_addr_s *a1,
               const struct ble_addr_s *a2,
               uint8_t *out);

/**
   Cryptographic toolbox @tt f6 function (2.2.8)

   This function is usually used for (secure) connection check.
 */
error_t ble_f6(const struct device_crypto_s *dev,
               const uint8_t *w,
               const uint8_t *n1, const uint8_t *n2,
               const uint8_t *r, uint32_t io_cap,
               const struct ble_addr_s *a1,
               const struct ble_addr_s *a2,
               uint8_t *out);

/**
   Cryptographic toolbox @tt g2 function (2.2.9)

   This function is usually used for (secure) passkey check.
 */
error_t ble_g2(const struct device_crypto_s *dev,
               const uint8_t *u, const uint8_t *v,
               const uint8_t *x, const uint8_t *y,
               uint32_t *out);

/**
   Key management diversifying function d1 function (Appendix B, B.2.1)

   This function is usually used for session key generation from LTK.
 */
error_t ble_d1(const uint8_t *k, uint16_t d, uint16_t r, uint8_t *out);

/**
   @this derives a session key from a LTK/STK and SKD.
 */
error_t ble_sk_derive(const uint8_t *lstk, const uint8_t *skd, uint8_t *sk);

/**
   @this is Specification-defined debug key for secure pairing
   testing.
 */
extern const struct ble_dh_key ble_dh_debug_key;

#endif
