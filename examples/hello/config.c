#include <mutek/printk.h>

#include <device/resources.h>
#include <device/irq.h>
# include <device/class/spi.h>
# include <device/class/gpio.h>
# include <device/class/mem.h>

#include <mutek/shell.h>
#include <mutek/mem_alloc.h>

static error_t lcd_cmd(const uint8_t *data, size_t size)
{
  struct device_spi_ctrl_s spi;
  struct device_gpio_s gpio;
  struct dev_spi_ctrl_transfer_s tr;
  error_t err;

  memset(&tr, 0, sizeof(tr));

  err = device_get_accessor_by_path(&gpio.base, NULL, "gpio*", DRIVER_CLASS_GPIO);
  assert(!err);

  err = device_get_accessor_by_path(&spi.base, NULL, "spi*", DRIVER_CLASS_SPI_CTRL);
  assert(!err);

  tr.cs_op = DEV_SPI_CS_NOP_NOP;
  tr.data.count = size;
  tr.data.in = NULL;
  tr.data.out = data;
  tr.data.in_width = 1;
  tr.data.out_width = 1;

  dev_gpio_out(&gpio, 56, 0);

  dev_gpio_out(&gpio, 51, 0);

  err = dev_spi_wait_transfer(&spi, &tr);
  assert(!err);

  dev_gpio_out(&gpio, 51, 1);

  device_put_accessor(&spi.base);
  device_put_accessor(&gpio.base);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(lcd_brightness)
{
  uint_fast16_t r = strto_uintl32(argv[0], NULL, 0);

  lcd_cmd((const uint8_t[]){0x81, r}, 2);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(lcd_rate)
{
  uint_fast16_t f = strto_uintl32(argv[0], NULL, 0);
  uint_fast16_t d = strto_uintl32(argv[1], NULL, 0);

  lcd_cmd((const uint8_t[]){0xd5, (f << 4) | d}, 2);

  termui_con_printf(con, "refresh rate: %d Hz\n", (80 + 5 * (f + 1)) / (d + 1));

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(lcd_scroll)
{
  uint_fast16_t f = strto_uintl32(argv[0], NULL, 0);

  lcd_cmd((const uint8_t[]){0x26, 0, 0, 0, 1, 0, 0xff}, 7);
  lcd_cmd((const uint8_t[]){0x2f}, 1);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(lcd_fill)
{
  uint_fast16_t n = strto_uintl32(argv[0], NULL, 0);

  size_t total = 128 * 64;
  reg_t nco = 0;
  uint8_t *buffer = mem_alloc(total / 8, mem_scope_sys);
  memset(buffer, 0, total / 8);

  for (size_t i = 0; i < total; ++i) {
    nco += n;

    if (nco < 100)
      continue;

    nco -= 100;
    buffer[i / 8] |= 1 << (i % 8);
  }

  struct device_mem_s mem;
  struct dev_mem_rq_s rq;
  error_t err;

  err = device_get_accessor_by_path(&mem.base, NULL, "lcd*", DRIVER_CLASS_MEM);
  assert(!err);

  rq.type = DEV_MEM_OP_PARTIAL_WRITE;
  rq.size = total / 8;
  rq.addr = 0;
  rq.data = buffer;
  rq.sc_log2 = 0;
  rq.band_mask = 1;

  err = dev_mem_wait_op(&mem, &rq);
  assert(!err);

  mem_free(buffer);

  device_put_accessor(&mem.base);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(lcd_fill2)
{
  uint_fast16_t n = strto_uintl32(argv[0], NULL, 0);

  size_t size = 128 * 64 / 8;
  size_t th = size * n / 100;
  uint8_t *buffer = mem_alloc(size, mem_scope_sys);

  memset(buffer, 0xff, th);
  memset(buffer + th, 0, size - th);

  struct device_mem_s mem;
  struct dev_mem_rq_s rq;
  error_t err;

  err = device_get_accessor_by_path(&mem.base, NULL, "lcd*", DRIVER_CLASS_MEM);
  assert(!err);

  rq.type = DEV_MEM_OP_PARTIAL_WRITE;
  rq.size = size;
  rq.addr = 0;
  rq.data = buffer;
  rq.sc_log2 = 0;
  rq.band_mask = 1;

  err = dev_mem_wait_op(&mem, &rq);
  assert(!err);

  mem_free(buffer);

  device_put_accessor(&mem.base);

  return 0;
}

static TERMUI_CON_GROUP_DECL(lcd_subgroup) =
{

  TERMUI_CON_ENTRY(lcd_rate, "rate",
                   TERMUI_CON_ARGS(2, 2)
  )

  TERMUI_CON_ENTRY(lcd_brightness, "brightness",
                   TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(lcd_scroll, "scroll",
                   TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(lcd_fill, "fill",
                   TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(lcd_fill2, "fill2",
                   TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(lcd_subgroup, "lcd")
