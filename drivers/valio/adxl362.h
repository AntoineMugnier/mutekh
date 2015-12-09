
#include <hexo/interrupt.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>
#include <device/class/valio.h>
#include <device/valio/motion.h>

enum adxl362_state_s
{
  ADXL362_STATE_DOWN,
  ADXL362_STATE_READY,
  ADXL362_STATE_READ,
  ADXL362_STATE_WRITE,
  ADXL362_STATE_WAIT,
};

#define R_CTX_PV 0

#define R_TMP0 1
#define R_TMP1 2
#define R_TMP2 3
#define R_TMP3 4
#define R_TMP4 5
#define R_TMP5 6

#define R_SIZE 7

#define R_STATUS 8
#define R_ARG0 9
#define R_ARG1 10
#define R_ARG2 11 

#define R_LINK 12

#define ADXL362_DEVID  0xAD
#define ADXL362_MEMSID 0x1D
#define ADXL362_PARTID 0xF2

#define ADXL362_RANGE_SELECTION        1  /* 1mg/bit */
#define ADXL362_TIME_GRANULARITY       165000  /* 166000 us */

#define ADXL362_FLAGS_BC_RUN  (1 << 0)  /* bytecode is running */
#define ADXL362_FLAGS_WAIT_RQ (1 << 1)  /* A wait request is pending */
#define ADXL362_FLAGS_IRQ     (1 << 2)  /* A irq is pending */

extern const struct bc_descriptor_s adxl362_bytecode;

struct adxl362_private_s
{
  uint32_t time;

  struct dev_irq_src_s src_ep;
  struct dev_spi_ctrl_rq_s spi_rq;

  /* queue for requests */
  dev_request_queue_root_t queue;

  /* cpu irq save mask */
  cpu_irq_state_t irq_save;

  gpio_id_t pin_map[1];

  uint8_t flags;
  int8_t x;
  int8_t y;
  int8_t z;
  enum adxl362_state_s state:8;
};

void adxl362_spi_entry_reset();
void adxl362_spi_entry_cfg();
void adxl362_spi_entry_wait();
void adxl362_spi_entry_irq();
void adxl362_spi_entry_read_value();

