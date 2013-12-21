
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/spi.h>

#ifndef CONFIG_DEVICE_SPI_REQUEST

static const uint8_t tx[7] = { 0x0f, 0xcc, 0xaa, 0x57, 0x28, 0x49, 0x35 };
static uint8_t rx[7] = {};

static DEVSPI_CTRL_TRANSFER_CALLBACK(tr_callback)
{
  printk("tr callback: %P\n", rx, sizeof(rx));
}

void main()
{
  struct device_spi_ctrl_s spi;

  device_dump_tree(0);

  if (device_get_accessor_by_path(&spi, NULL, "spi*", DRIVER_CLASS_SPI_CTRL))
    abort();

  printk("spi found\n");

  static struct dev_spi_ctrl_config_s cfg = {
    .bit_rate = 100000,
    .word_width = 8,
  };

  if (DEVICE_OP(&spi, config, &cfg))
    abort();

  printk("spi config done\n");

  static struct dev_spi_ctrl_transfer_s tr = {
    .count = sizeof(tx),
    .in = rx,
    .out = tx,
    .callback = tr_callback,
    .in_width = sizeof (rx[0]),
    .out_width = sizeof (tx[0]),
  };

  if (DEVICE_OP(&spi, transfer, &tr, 1))
    abort();

  while (1)
    ;
}

#else

static const bc_opcode_t bytecode[] =
  {
    BC_CST8(0, 0x55),
    BC_SPI_SWP(0, 1),
    BC_DUMP(),
    BC_END(),
  };

void main()
{
  struct device_spi_ctrl_s spi;

  device_dump_tree(0);

  if (device_get_accessor_by_path(&spi, NULL, "spi*", DRIVER_CLASS_SPI_CTRL))
    abort();

  printk("spi found\n");

  struct dev_spi_ctrl_request_s rq = {
    .config = {
      .bit_rate = 100000,
      .word_width = 8,
      .miso_pol = DEV_SPI_CS_ACTIVE_LOW,
    },
    .cs_id = 0,
    .slave_id = 0,
    .delay_unit = 1,
  };

  bc_init(&rq.vm, bytecode, sizeof(bytecode), 1, 0);

  device_init_accessor(&rq.gpio);

  error_t err = dev_spi_request_start(&spi, &rq);

  printk("done! %i\n", err);
}

#endif
