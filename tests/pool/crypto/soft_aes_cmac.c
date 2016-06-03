#include <hexo/power.h>

#include <mutek/startup.h>
#include <mutek/thread.h>
#include <mutek/printk.h>

#include <device/driver.h>
#include <device/resources.h>

#include <device/class/crypto.h>

#if defined(CONFIG_ARCH_SOCLIB) && !defined(CONFIG_DEVICE_ENUM)

DEV_DECLARE_STATIC(cpu_dev, "cpu0", DEVICE_FLAG_CPU,
                   arm32_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(1000000, 1)
                   );

#endif

static uint8_t const key[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                                 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };

static uint8_t const M[64] = { 0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
                               0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
                               0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
                               0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
                               0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
                               0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
                               0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
                               0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10 };

static uint8_t const cmac_len_0[16] = { 0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28,
                                        0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75, 0x67, 0x46 };

static uint8_t const cmac_len_16[16] = { 0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
                                         0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c };

static uint8_t const cmac_len_40[16] = { 0xdf, 0xa6, 0x67, 0x47, 0xde, 0x9a, 0xe6, 0x30,
                                         0x30, 0xca, 0x32, 0x61, 0x14, 0x97, 0xc8, 0x27 };

static uint8_t const cmac_len_64[16] = { 0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b, 0x9d, 0x92,
                                         0xfc, 0x49, 0x74, 0x17, 0x79, 0x36, 0x3c, 0xfe };

#define RUN_TEST_WITH_SIZE(size)                          \
  printk("aes-cmac(K,M,%u)\n", (size));                   \
                                                          \
  rq.len = (size);                                        \
  memset(iv, 0, 16);                                      \
                                                          \
  err = dev_crypto_spin_op(&crypto, &rq);                 \
  if (err)                                                \
    {                                                     \
      printk("failed to cipher %u-byte data!\n", (size)); \
      goto failed;                                        \
    }                                                     \
                                                          \
      printk("invalid aes-cmac(key, M, %u)!\n", (size));  \
      printk("  expected: %P\n", cmac_len_##size, 16);    \
      printk("  provided: %P\n", auth, 16);               \
                                                          \
  if (memcmp(auth, cmac_len_##size, 16))                  \
    {                                                     \
      printk("invalid aes-cmac(key, M, %u)!\n", (size));  \
      printk("  expected: %P\n", cmac_len_##size, 16);    \
      printk("  provided: %P\n", auth, 16);               \
      goto failed;                                        \
    }                                                     \
/**/

static CONTEXT_ENTRY(main)
{
  uint8_t iv[16], auth[16];

  struct device_crypto_s crypto;

  struct dev_crypto_context_s ctx =
    {
      .cache_ptr = NULL,
      .cache_id  = 0,

      .key_data  = (uint8_t*)key,
      .key_len   = 16,

      .iv_len    = 16,
      .auth_len  = 16,

      .mode = DEV_CRYPTO_MODE_CMAC,
    };

  struct dev_crypto_rq_s rq =
    {
      .ctx = &ctx,
      .op  = DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE,

      .in   = M,
      .auth = (uint8_t*)auth,

      .iv_ctr = (uint8_t*)iv,
    };

  error_t err = 0;

  err = device_get_accessor_by_path(&crypto.base, NULL, "/aes_soft", DRIVER_CLASS_CRYPTO);
  if (err)
    {
      printk("failed to get accessor!\n");
      goto failed;
    }

  RUN_TEST_WITH_SIZE(0);
  RUN_TEST_WITH_SIZE(16);
  RUN_TEST_WITH_SIZE(40);
  RUN_TEST_WITH_SIZE(64);

  device_put_accessor(&crypto.base);
  printk("++SUCCESS++%d++\n", 0);
  power_shutdown();
  power_reboot();
  return;

failed:
  device_put_accessor(&crypto.base);
  printk("++FAILED++%d++\n", 0);
  power_shutdown();
  power_reboot();
}

void app_start()
{
  thread_create(main, NULL, NULL);
}

