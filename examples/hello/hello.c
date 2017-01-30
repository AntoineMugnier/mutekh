
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <mutek/printk.h>
#include <device/class/mem.h>
#include <device/device.h>
#include <hexo/iospace.h>

#include "dac.h"

static const uint8_t logo[128*64/8] = {
#include "logo.h"
};

static CONTEXT_ENTRY(main)
{
  struct device_mem_s mem;
  struct dev_mem_rq_s rq;
  error_t err;

  size_t count = 0;

  for (size_t i = 0; i < sizeof(logo); ++i) {
    uint8_t d = logo[i];
    while (d) {
      count += d & 1;
      d >>= 1;
    }
  }

  printk("%d pixels on\n", count);

  err = device_get_accessor_by_path(&mem.base, NULL, "lcd*", DRIVER_CLASS_MEM);
  assert(!err);

  rq.type = DEV_MEM_OP_PARTIAL_WRITE;
  rq.size = sizeof(logo);
  rq.addr = 0;
  rq.data = logo;
  rq.sc_log2 = 0;
  rq.band_mask = 1;

  err = dev_mem_wait_op(&mem, &rq);
  if (err) {
    printk("Write failed: %d\n", rq.err);
  }
}

void app_start()
{
  cpu_mem_write_32(EFM32_CMU_ADDR + 0x44, cpu_mem_read_32(EFM32_CMU_ADDR + 0x44) | 0x20000);
  cpu_mem_write_32(EFM32_DAC0_ADDR + EFM32_DAC_CTRL, 0x210);
  cpu_mem_write_32(EFM32_DAC0_ADDR + EFM32_DAC_CH0CTRL, 0x3);
  cpu_mem_write_32(EFM32_DAC0_ADDR + EFM32_DAC_CH0DATA, 0);

  assert(!thread_create(main, 0, NULL));
}

#include <device/resources.h>
#include <device/class/spi.h>

DEV_DECLARE_STATIC(lcd, "lcd", 0, sd1306_drv,
                   DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
                   DEV_STATIC_RES_DEV_TIMER("rtc* timer*"),
                   DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
#if 0
                   DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", 24),
                   DEV_STATIC_RES_GPIO("reset", 23, 1),
                   DEV_STATIC_RES_GPIO("dc", 20, 1),
#else
                   DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", 51),
                   DEV_STATIC_RES_GPIO("reset", 55, 1),
                   DEV_STATIC_RES_GPIO("dc", 56, 1),
#endif
                   );
