
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/class/spi.h>
#include <hexo/iospace.h>

#ifndef CONFIG_DEVICE_SPI_REQUEST

static const uint8_t tx[7] = { 0x0f, 0xcc, 0xaa, 0x57, 0x28, 0x49, 0x35 };
static uint8_t rx[7] = {};

static KROUTINE_EXEC(tr_callback)
{
  struct dev_spi_ctrl_transfer_s *tr = KROUTINE_CONTAINER(kr, *tr, kr);

  printk("tr callback: err=%i, %P\n", tr->err, rx, sizeof(rx));
}

void main()
{
  struct device_spi_ctrl_s spi;

  if (device_get_accessor_by_path(&spi.base, NULL, "usart1", DRIVER_CLASS_SPI_CTRL))
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
    .in_width = sizeof (rx[0]),
    .out_width = sizeof (tx[0]),
  };

  kroutine_init_immediate(&tr.kr, &tr_callback);

  DEVICE_OP(&spi, transfer, &tr);

  while (1)
    ;
}

#else

static const bc_opcode_t bytecode[] =
  {
    BC_SPI_GPIOMODE(66, DEV_GPIO_OUTPUT),
    BC_CST16(3, 10000),
    BC_CST8(0, 0x55),
  /* label:re */
    BC_ADD8(0, 1),
    BC_SPI_SWP(0, 1),
    // BC_DUMP(),
    BC_CST32(2, 100000),
    BC_SPI_YIELD_DELAY(2),
    BC_SPI_GPIOSET(66, 0),
    BC_LOOP(3, -8 /* goto:re */),
    BC_END(),
  };

static KROUTINE_EXEC(rq_callback)
{
  struct dev_spi_ctrl_request_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);

  printk("rq callback: err=%i\n", rq->err);
}

void main()
{
  struct device_spi_ctrl_s spi;

  if (device_get_accessor_by_path(&spi.base, NULL, "usart1", DRIVER_CLASS_SPI_CTRL))
    abort();

  printk("spi found\n");

  struct dev_spi_ctrl_request_s rq = {
    .config = {
      .bit_rate = 100000,
      .word_width = 8,
      //      .miso_pol = DEV_SPI_ACTIVE_LOW,
    },
    .cs_id = 0,
    .cs_gpio = 1,
  };

  bc_init(&rq.vm, bytecode, sizeof(bytecode), 1, 0);

  device_init_accessor(&rq.gpio);

  if (device_get_accessor_by_path(&rq.gpio.base, NULL, "gpio*", DRIVER_CLASS_GPIO))
    abort();

  kroutine_init_immediate(&rq.kr, &rq_callback);

  DEVICE_OP(&spi, request, &rq);

  while (1)
    ;

  printk("done! %i\n", rq.err);
}


#endif
