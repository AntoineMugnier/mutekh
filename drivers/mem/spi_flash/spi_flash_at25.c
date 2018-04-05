
#include "spi_flash.h"
#include "spi_flash_spi.o.h"

static DEV_INIT(spi_flash_init)
{
  static const struct spi_flash_info_s info_at25 = {
    .mem_info = {
      .type = DEV_MEM_FLASH,
      .flags = DEV_MEM_PARTIAL_READ | DEV_MEM_PARTIAL_WRITE
             | DEV_MEM_PAGE_READ | DEV_MEM_PAGE_WRITE
             | DEV_MEM_ERASE_ONE | DEV_MEM_CROSS_READ,

      /* ~100 000 */
      .erase_cycles_p = 14,
      .erase_cycles_m = 6,

      .page_log2 = 8,
      .erase_log2 = 12,
    },
    .spi_cfg = {
        .ck_mode = DEV_SPI_CK_MODE_0,
	.bit_order = DEV_SPI_MSB_FIRST,
	.miso_pol = DEV_SPI_ACTIVE_HIGH,
	.mosi_pol = DEV_SPI_ACTIVE_HIGH,
	.cs_pol = DEV_SPI_ACTIVE_LOW,
	.bit_rate1k = 8000,
	.word_width = 8,
    },
    .send_header = &spi_flash_bc_send_header_3bytes,
    .ops_exclude = 0,
    .byte_erase_delay = 30,
    .byte_write_delay = 7,
    .chip_erase_delay = 8
  };

  return spi_flash_init_common(dev, &info_at25);
}

DRIVER_DECLARE(spi_flash_at25_drv, 0, "SPI at25 flash", spi_flash,
               DRIVER_MEM_METHODS(spi_flash) );

DRIVER_REGISTER(spi_flash_at25_drv);

