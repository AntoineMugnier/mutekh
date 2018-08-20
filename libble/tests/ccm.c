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
#include <mutek/mem_alloc.h>
#include <string.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/crypto.h>
#include <ble/ccm_params.h>

#include "check.h"

static void ccm_test_core(struct device_crypto_s *aes)
{
  struct dev_crypto_context_s ctx;
  struct dev_crypto_rq_s rq;
  error_t err;
  struct ble_ccm_state_s ccm_state = {};
  uint8_t out_[138];
  uint8_t *out = out_ + 4;
  uint8_t in_[138];
  uint8_t *in = in_ + 4;

  const uint8_t iv[] = { 0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE, };
  const uint8_t sk[] = { 0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
                         0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66, };

  struct dev_crypto_info_s crypto_info;

  memset(&rq, 0, sizeof(rq));
  
  DEVICE_OP(aes, info, &crypto_info);

  ctx.mode = DEV_CRYPTO_MODE_BLE_CCM;
  ctx.state_data = mem_alloc(crypto_info.state_size, mem_scope_sys);
  ctx.key_data = (void*)sk;
  ctx.key_len = 16;
  ctx.iv_len = 8;
  ctx.auth_len = 2;
  ctx.encrypt_only = 1;

  rq.op = DEV_CRYPTO_INIT;
  rq.ctx = &ctx;
  rq.iv_ctr = (void*)iv;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  // START_ENC_RSP1
  
  rq.op = DEV_CRYPTO_FINALIZE;
  rq.ctx = &ctx;
  rq.out = out;
  rq.in = in;
  rq.iv_ctr = (void*)&ccm_state;

  memcpy(in, (const uint8_t[]){ 0x0f, 0x01, 0x06 }, 3);
  rq.len = 3;
  ccm_state.sent_by_master = 1;
  ccm_state.packet_counter = 0;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  CHECK("START_ENC_RSP1 Encrypt", out,  0x0f,  0x05,  0x9f,  0xcd,  0xa7,  0xf4,  0x48, );

  rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
  rq.ctx = &ctx;

  memcpy(in, (const uint8_t[]){ 0x0f,  0x05,  0x9f,  0xcd,  0xa7,  0xf4,  0x48, }, 7);
  rq.len = 7;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  CHECK("START_ENC_RSP1 Decrypt", out,  0x0f,  0x01,  0x06, );

  // START_ENC_RSP2
  
  rq.op = DEV_CRYPTO_FINALIZE;
  rq.ctx = &ctx;

  memcpy(in, (const uint8_t[]){ 0x07, 0x01, 0x06 }, 3);
  rq.len = 3;
  ccm_state.sent_by_master = 0;
  ccm_state.packet_counter = 0;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  CHECK("START_ENC_RSP2 Encrypt", out,  0x07,  0x05, 0xa3,  0x4c,  0x13,  0xa4,  0x15, );

  rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
  rq.ctx = &ctx;
  rq.out = out;
  memcpy(in, (const uint8_t[]){ 0x07,  0x05, 0xa3,  0x4c,  0x13,  0xa4,  0x15, }, 7);
  rq.len = 7;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  CHECK("START_ENC_RSP2 Decrypt", out,  0x07,  0x01,  0x06, );

  // Data packet1
  
  rq.op = DEV_CRYPTO_FINALIZE;
  rq.ctx = &ctx;

  memcpy(in, (const uint8_t[]){
      0x0E, 0x1b, 0x17, 0x00, 0x63, 0x64, 0x65, 0x66,
        0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e,
        0x6f, 0x70, 0x71, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, }, 29);
  rq.len = 29;
  ccm_state.sent_by_master = 1;
  ccm_state.packet_counter = 1;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  CHECK("Data packet1 Encrypt", out,
        0x0E, 0x1F, 0x7A, 0x70, 0xD6, 0x64, 0x15, 0x22,
        0x6D, 0xF2, 0x6B, 0x17, 0x83, 0x9A, 0x06, 0x04,
        0x05, 0x59, 0x6B, 0xD6, 0x56, 0x4F, 0x79, 0x6B,
        0x5B, 0x9C, 0xE6, 0xFF, 0x32, 0xF7, 0x5A, 0x6D,
        0x33,
        );

  rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
  rq.ctx = &ctx;

  memcpy(in, (const uint8_t[]){
      0x0E, 0x1F, 0x7A, 0x70, 0xD6, 0x64, 0x15, 0x22,
        0x6D, 0xF2, 0x6B, 0x17, 0x83, 0x9A, 0x06, 0x04,
        0x05, 0x59, 0x6B, 0xD6, 0x56, 0x4F, 0x79, 0x6B,
        0x5B, 0x9C, 0xE6, 0xFF, 0x32, 0xF7, 0x5A, 0x6D,
        0x33, }, 33);
  rq.len = 33;

  err = dev_crypto_wait_rq(aes, &rq);
  ensure(!err);

  CHECK("Data packet1 Decrypt", out,
        0x0E, 0x1b, 0x17, 0x00, 0x63, 0x64, 0x65, 0x66,
        0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e,
        0x6f, 0x70, 0x71, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, );

  printk("Checking CCM for sizes 1..128\n");
  for (size_t len = 1; len < 128; ++len) {
    uint8_t tmp[256];
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 1;
    rq.op = DEV_CRYPTO_FINALIZE;
    rq.ctx = &ctx;
    memcpy(in + 2, ccm_test_core, len);
    in[0] = 0xe;
    in[1] = len;
    rq.in = in;
    rq.out = tmp + 4;
    rq.len = len;
    err = dev_crypto_wait_rq(aes, &rq);
    ensure(!err);
    ensure(tmp[4] == 0xe);
    ensure(tmp[5] == len + 4);
    rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
    rq.in = tmp + 4;
    rq.out = out;
    rq.len = len + 4;
    err = dev_crypto_wait_rq(aes, &rq);
    ensure(!err);
    ensure(!memcmp(in, out, len));
  }

  printk("Ensuring MIC is actually checked\n");
  for (size_t len = 1; len < 128; ++len) {
    uint8_t tmp[256];
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 1;
    rq.op = DEV_CRYPTO_FINALIZE;
    rq.ctx = &ctx;
    memcpy(in + 2, ccm_test_core, len);
    in[0] = 0xe;
    in[1] = len;
    rq.in = in;
    rq.out = tmp + 4;
    rq.len = len;
    err = dev_crypto_wait_rq(aes, &rq);
    ensure(!err);
    ensure(tmp[4] == 0xe);
    ensure(tmp[5] == len + 4);
    tmp[6] ^= len;
    rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
    rq.in = tmp + 4;
    rq.out = out;
    rq.len = len + 4;
    err = dev_crypto_wait_rq(aes, &rq);
    ensure(err);
  }
}

void ccm_test(void)
{
  struct device_crypto_s aes;

  if (device_get_accessor_by_path(&aes, NULL, "aes*", DRIVER_CLASS_CRYPTO) == 0)
    goto found;

  printk("aes not found\n");
  abort();

 found:
  ccm_test_core(&aes);

  device_put_accessor(&aes);
}
