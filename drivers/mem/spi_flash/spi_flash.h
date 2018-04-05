
#include <mutek/bytecode.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/spi.h>
#include <device/class/mem.h>

enum spi_flash_state_e
{
  SPI_FLASH_STATE_INIT,
  SPI_FLASH_STATE_IDLE,
  SPI_FLASH_STATE_BUSY,
  SPI_FLASH_STATE_SLEEP,
};

DRIVER_PV(struct spi_flash_private_s
{
  dev_request_queue_root_t queue;
  struct device_spi_ctrl_s spi;
  struct dev_spi_ctrl_bytecode_rq_s srq;
  const struct spi_flash_info_s *info;
  uintptr_t size;
  dev_timer_delay_t byte_erase_delay;
  dev_timer_delay_t byte_write_delay;
  dev_timer_delay_t chip_erase_delay;
  enum spi_flash_state_e state;
});

#define SPI_FLASH_CMD_WR 0x02
#define SPI_FLASH_CMD_RD 0x03
#define SPI_FLASH_CMD_WR_DIS 0x04
#define SPI_FLASH_CMD_WR_EN  0x06
#define SPI_FLASH_CMD_RD_STATUS 0x05
#define SPI_FLASH_CMD_PAGE_ERASE 0x20
#define SPI_FLASH_CMD_CHIP_ERASE 0x60
#define SPI_FLASH_CMD_ID 0x9f
#define SPI_FLASH_CMD_SLEEP 0xb9
#define SPI_FLASH_CMD_WAKE 0xab

#define SPI_FLASH_ST_BUSY 0x01
#define SPI_FLASH_ST_WEL  0x02

#define SPI_FLASH_PAGE_SIZE 256
#define SPI_FLASH_ERASE_SIZE 4096

struct spi_flash_info_s
{
  struct dev_spi_ctrl_config_s spi_cfg;

  /* reported by spi_flash_info */
  struct dev_mem_info_s mem_info;

  /* default byte size of the flash, or 0 */
  uint32_t size;

  /* pointer to bytecode function used to send command header
     containing the address */
  void *send_header;
  enum dev_mem_rq_type_e ops_exclude;

  /* delays in us/byte, will be tried 3 times */
  uint8_t byte_erase_delay;
  uint8_t byte_write_delay;
  uint8_t chip_erase_delay;
};

error_t spi_flash_init_common(struct device_s *dev,
			      const struct spi_flash_info_s *info);

DEV_CLEANUP(spi_flash_cleanup);
DEV_MEM_REQUEST(spi_flash_request);
DEV_MEM_INFO(spi_flash_info);
DEV_USE(spi_flash_use);
