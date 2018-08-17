#include <assert.h>

#include <mutek/thread.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>
#include <hexo/iospace.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/timer.h>
#include <device/class/crypto.h>

#include <ble/ccm_params.h>

static bool_t loop_done = 0;

static KROUTINE_EXEC(done2)
{
  loop_done = 1;
}

#define BENCH(text, init, iter)                                         \
    {                                                                   \
        init;                                                           \
                                                                        \
        loop_done = 0;                                                  \
                                                                        \
        printk("%30s", text);          \
                                                                        \
        dev_timer_init_sec(&timer, &timer_req.delay, NULL, ms, 1000);   \
        dev_timer_rq_init_immediate(&timer_req.base.kr, done2);           \
        DEVICE_OP(&timer, request, &timer_req);                         \
                                                                        \
        uint32_t iter_count = 0;                                        \
                                                                        \
        while (!loop_done) {                                            \
            iter_count++;                                               \
            {iter;}                                                     \
            order_compiler_mem();                                       \
        }                                                               \
                                                                        \
        uint32_t cycles = ms * hz / 1000 / iter_count;                  \
        uint32_t us = ms * 1000 / iter_count;                           \
                                                                        \
        printk(" %8d %4d %6d\n", iter_count, us, cycles);               \
    }

void speed_test(void)
{
  const uint32_t ms = 400;
  const uint32_t hz = 16000000;
  struct device_timer_s timer;
  struct device_crypto_s aes;
  struct device_crypto_s rng;
  struct dev_timer_rq_s timer_req;

  if (device_get_accessor_by_path(&timer, NULL, "timer1", DRIVER_CLASS_TIMER)) {
    printk("timer1 not found\n");
    abort();
  }

  if (device_get_accessor_by_path(&aes, NULL, "aes", DRIVER_CLASS_CRYPTO)) {
    printk("aes not found\n");
    abort();
  }

  if (device_get_accessor_by_path(&rng, NULL, "rng", DRIVER_CLASS_CRYPTO)) {
    printk("rng not found\n");
    abort();
  }

  device_start(&timer);
  dev_timer_init_sec(&timer, &timer_req.delay, NULL, ms, 1000);

  printk("Starting %d ms benches\n", ms);
  printk("%30s %8s %4s %6s\n", "bench", "iter", "us", "cycles");

  BENCH("empty",,);

  BENCH("AES-128 encrypt",
        struct dev_crypto_context_s ctx;
        uint8_t data[48];
        struct dev_crypto_rq_s rq;

        ctx.mode = DEV_CRYPTO_MODE_ECB;
        ctx.cache_ptr = NULL;
        ctx.key_data = data;
        ctx.key_len = 16;

        rq.op = DEV_CRYPTO_INIT;
        rq.len = 16;
        rq.ctx = &ctx;
        rq.in = data + 16;
        rq.out = data + 32;
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 16B rand",
        uint8_t data[16];
        struct dev_crypto_rq_s rq;

        rq.op = DEV_CRYPTO_FINALIZE;
        rq.ctx = NULL;
        rq.len = sizeof(data);
        rq.out = data;
        ,
        dev_crypto_spin_op(&rng, &rq);
        );

  BENCH("AES DRBG 16B aligned seed",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_INVERSE;
        rq.ctx = &ctx;
        rq.ad = (void*)((uintptr_t)speed_test & ~3);
        rq.ad_len = 16;
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 32B aligned seed",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_INVERSE;
        rq.ctx = &ctx;
        rq.ad = (void*)((uintptr_t)speed_test & ~3);
        rq.ad_len = 32;
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 64B aligned seed",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_INVERSE;
        rq.ctx = &ctx;
        rq.ad = (void*)((uintptr_t)speed_test & ~3);
        rq.ad_len = 64;
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 128B aligned seed",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_INVERSE;
        rq.ctx = &ctx;
        rq.ad = (void*)((uintptr_t)speed_test & ~3);
        rq.ad_len = 128;
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 128B unaligned seed",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_INVERSE;
        rq.ctx = &ctx;
        rq.ad = (void*)(((uintptr_t)speed_test & ~3) + 1);
        rq.ad_len = 128;
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 8B rand",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];
        uint8_t out[8];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_FINALIZE;
        rq.ctx = &ctx;
        rq.out = out;
        rq.len = sizeof(out);
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 16B rand",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];
        uint8_t out[16];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_FINALIZE;
        rq.ctx = &ctx;
        rq.out = out;
        rq.len = sizeof(out);
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  BENCH("AES DRBG 32B rand",
        struct dev_crypto_context_s ctx;
        struct dev_crypto_rq_s rq;
        uint32_t data[8];
        uint8_t out[32];

        ctx.mode = DEV_CRYPTO_MODE_RANDOM;
        ctx.cache_ptr = NULL;
        ctx.state_data = data;

        rq.op = DEV_CRYPTO_FINALIZE;
        rq.ctx = &ctx;
        rq.out = out;
        rq.len = sizeof(out);
        ,
        dev_crypto_spin_op(&aes, &rq);
        );

  {
    struct dev_crypto_context_s ctx;
    struct dev_crypto_rq_s rq;
    error_t err;
    uint8_t out[80];
    uint8_t in[80] = {0, 0x0f, 0x01, 0x06};
    const uint8_t iv[] = { 0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE, };
    const uint8_t sk[] = { 0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
                           0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66, };
    struct dev_crypto_info_s crypto_info;
    struct ble_ccm_state_s ccm_state = {};
        
    memset(&rq, 0, sizeof(rq));
  
    DEVICE_OP(&aes, info, &crypto_info);

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

    err = dev_crypto_wait_op(&aes, &rq);
    ensure(!err);
  
    rq.op = DEV_CRYPTO_FINALIZE;
    rq.ctx = &ctx;
    rq.out = out;
    rq.in = in;
    rq.iv_ctr = (void*)&ccm_state;

    memcpy(in, (const uint8_t[]){ 0x0f, 0x01, 0x06 }, 3);
    rq.len = 3;
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 0;

    BENCH("BLE AES-CCM 1B encrypt",
          ,
          dev_crypto_spin_op(&aes, &rq);
          );

    memcpy(in, (const uint8_t[]){
        0x0E, 0x1b, 0x17, 0x00, 0x63, 0x64, 0x65, 0x66,
          0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e,
          0x6f, 0x70, 0x71, 0x31, 0x32, 0x33, 0x34, 0x35,
          0x36, 0x37, 0x38, 0x39, 0x30, }, 29);
    rq.len = 29;
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 1;

    BENCH("BLE AES-CCM 27B encrypt",
          ,
          dev_crypto_spin_op(&aes, &rq);
          );

    memcpy(in, (const uint8_t[]){
        0x0E, 0x1b, 0x17, 0x00, 0x63, 0x64, 0x65, 0x66,
          0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e,
          0x6f, 0x70, 0x71, 0x31, 0x32, 0x33, 0x34, 0x35,
          0x36, 0x37, 0x38, 0x39, 0x30, 0xa2 }, 30);
    rq.len = 30;
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 1;

    BENCH("BLE AES-CCM 28B encrypt",
          ,
          dev_crypto_spin_op(&aes, &rq);
          );

  }

  {
    struct dev_crypto_context_s ctx;
    struct dev_crypto_rq_s rq;
    error_t err;
    uint8_t out[80];
    uint8_t in[80] = {0, 0x0f, 0x01, 0x06};
    const uint8_t iv[] = { 0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE, };
    const uint8_t sk[] = { 0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
                           0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66, };
    struct dev_crypto_info_s crypto_info;
    struct ble_ccm_state_s ccm_state = {};
        
    memset(&rq, 0, sizeof(rq));
  
    DEVICE_OP(&aes, info, &crypto_info);

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

    err = dev_crypto_wait_op(&aes, &rq);
    ensure(!err);
  
    rq.op = DEV_CRYPTO_FINALIZE | DEV_CRYPTO_INVERSE;
    rq.ctx = &ctx;
    rq.out = out;
    rq.in = in;
    rq.iv_ctr = (void*)&ccm_state;

    memcpy(in, (const uint8_t[]){ 0x0f, 0x05, 0x9f, 0xcd, 0xa7, 0xf4, 0x48 }, 7);
    rq.len = 7;
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 0;

    BENCH("BLE AES-CCM 1B decrypt",
          ,
          dev_crypto_spin_op(&aes, &rq);
          );

    memcpy(in, (const uint8_t[]){
        0x0E, 0x1F, 0x7A, 0x70, 0xD6, 0x64, 0x15, 0x22,
          0x6D, 0xF2, 0x6B, 0x17, 0x83, 0x9A, 0x06, 0x04,
          0x05, 0x59, 0x6B, 0xD6, 0x56, 0x4F, 0x79, 0x6B,
          0x5B, 0x9C, 0xE6, 0xFF, 0x32, 0xF7, 0x5A, 0x6D,
          0x33, }, 33);
    rq.len = 33;
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 1;

    BENCH("BLE AES-CCM 27B decrypt",
          ,
          dev_crypto_spin_op(&aes, &rq);
          );

    memcpy(in, (const uint8_t[]){
        0x0e, 0x1f, 0x7a, 0x70, 0xd6, 0x64, 0x15, 0x22,
          0x6d, 0xf2, 0x6b, 0x17, 0x83, 0x9a, 0x06, 0x04,
          0x05, 0x59, 0x6b, 0xd6, 0x56, 0x4f, 0x79, 0x6b,
          0x5b, 0x9c, 0xe6, 0xff, 0x32, 0xf7, 0x5a, 0x6d,
          0x33, 0x56,
          }, 34);
    rq.len = 34;
    ccm_state.sent_by_master = 1;
    ccm_state.packet_counter = 1;

    BENCH("BLE AES-CCM 28B decrypt",
          ,
          dev_crypto_spin_op(&aes, &rq);
          );

    
      }
}
