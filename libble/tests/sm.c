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

#include <assert.h>

#include <mutek/printk.h>
#include <string.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/timer.h>
#include <device/class/crypto.h>

#include <ble/crypto.h>
#include <ble/protocol/address.h>

#include "check.h"

static void crypto_test_c1(const struct device_crypto_s *aes)
{
  const struct ble_addr_s ia = {
    .type = BLE_ADDR_RANDOM,
    .addr = { 0xa6, 0xa5, 0xa4, 0xa3, 0xa2, 0xa1, },
  };

  const struct ble_addr_s ra = {
    .type = BLE_ADDR_PUBLIC,
    .addr = { 0xb6, 0xb5, 0xb4, 0xb3, 0xb2, 0xb1, },
  };

  const uint8_t k[16] = {};
  const uint8_t pres[7] = {0x02, 0x03, 0x00, 0x00, 0x08, 0x00, 0x05};
  const uint8_t preq[7] = {0x01, 0x01, 0x00, 0x00, 0x10, 0x07, 0x07};
  const uint8_t r[16] = {
    0xE0, 0x2E, 0x70, 0xC6, 0x4E, 0x27, 0x88, 0x63,
    0x0E, 0x6F, 0xAD, 0x56, 0x21, 0xD5, 0x83, 0x57,
  };
  uint8_t out[16];

  ble_c1(aes, k, r, preq, pres, &ia, &ra, out);

  CHECK("c1", out,
        0x86, 0x3b, 0xf1, 0xbe, 0xc5, 0x4d, 0xa7, 0xd2,
        0xea, 0x88, 0x89, 0x87, 0xef, 0x3f, 0x1e, 0x1e);
}

static void crypto_test_s1(const struct device_crypto_s *aes)
{
  const uint8_t k[16] = {};
  const uint8_t r1[] = {0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
                        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x00};
  const uint8_t r2[] = {0x00, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99,
                        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};
  uint8_t out[16];

  ble_s1(aes, k, r1, r2, out);

  CHECK("s1", out,
        0x62, 0xa0, 0x6d, 0x79, 0xae, 0x16, 0x42, 0x5b,
        0x9b, 0xf4, 0xb0, 0xe8, 0xf0, 0xe1, 0x1f, 0x9a);
}

static void crypto_test_ah(const struct device_crypto_s *aes)
{
  const uint8_t k[16] = {0x9b, 0x7d, 0x39, 0x0a, 0xa6, 0x10, 0x10, 0x34,
                         0x05, 0xad, 0xc8, 0x57, 0xa3, 0x34, 0x02, 0xec};
  const uint8_t i[3] = {0x94, 0x81, 0x70};
  uint8_t out[3] = {};

  ble_ah(aes, k, i, out);

  CHECK("ah", out, 0xaa, 0xfb, 0x0d);
}

void sm_test(void)
{
  struct device_crypto_s aes;

  if (device_get_accessor_by_path(&aes, NULL, "aes", DRIVER_CLASS_CRYPTO) == 0)
    goto found;

  if (device_get_accessor_by_path(&aes, NULL, "aes_soft", DRIVER_CLASS_CRYPTO) == 0)
    goto found;

  printk("aes not found\n");
  abort();

 found:
  crypto_test_c1(&aes);
  crypto_test_ah(&aes);
  crypto_test_s1(&aes);

  device_put_accessor(&aes);
}
