#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <hexo/iospace.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>

#include <device/class/spi.h>
#include <device/class/dma.h>
#include <device/class/timer.h>
#include <device/class/gpio.h>
#include <device/class/rfpacket.h>
#include <device/class/iomux.h>
#include <device/class/bitbang.h>

#include "sx1276_regs_fsk.h"

//#define CONFIG_DRIVER_RFPACKET_SX127X_DEBUG
//#define SX127X_PRINT_REGS

#define SX127X_PIN_COUNT 3
#define SX127X_IO_RST  0
#define SX127X_DIO0    1
#define SX127X_DIO4    2

#define STATUS          15
#define R_CTX_PV        14

#define CMD_FREQ_CHANGE      0x10000
#define CMD_RX_SCANNING      0x20000
#define CMD_RXC_RAW          0x40000

#define STATUS_IRQ_MSK          0xFF
#define STATUS_INFO_MSK         0x00FFFF00
#define STATUS_CRC_VALID        9
#define STATUS_RX_PAYLOAD_RDY   10
#define STATUS_TX_SENT          11
#define STATUS_OTHER_ERR        16

#define _MSK(idx)               (1 << idx)

#define SX127X_VERSION_VALUE 0x12
#define SX127X_FREQ_TAB_COUNT 0x2

#define IO_MAX_PREAMBLE_SIZE 512

#define SX127X_TS_RE_US 150
#define SX127X_TS_FS_US 60

#define SX127X_RSSI_OFFSET SX1276_RSSICONFIG_OFFSET_P_00_DB

BC_CCALL_FUNCTION(sx127x_alloc);
BC_CCALL_FUNCTION(sx127x_next_hopping_freq);

enum sx127x_state_e
{
  SX127X_STATE_INITIALISING,
  SX127X_STATE_IDLE,
  SX127X_STATE_SLEEP,
  /* Configuring Radio transceiver for any operation except RX continuous */
  SX127X_STATE_CONFIG,
  /* Configuring Radio transceiver for RX continuous */
  SX127X_STATE_CONFIG_RXC,
  SX127X_STATE_CONFIG_RXC_PENDING_STOP,
  SX127X_STATE_RX,      /* 6 */
  /* Rx continous.*/
  SX127X_STATE_RXC,
  /* Rx continuous on multiple channels */
  SX127X_STATE_RX_SCANNING,
  /* A stop rx continous is pending */  
  SX127X_STATE_RXC_PENDING_STOP,
  /* Rx continous is stopping */  
  SX127X_STATE_RXC_STOP, 
  SX127X_STATE_TX,
  SX127X_STATE_TX_FAIR,
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
  SX127X_STATE_RXC_RAW_ALLOC,                           /* 13 */
  SX127X_STATE_RXC_RAW,
  SX127X_STATE_RXC_RAW_PENDING_STOP,
  SX127X_STATE_RXC_RAW_STOP,
  SX127X_STATE_RX_RAW_ALLOC,                       
  SX127X_STATE_RX_RAW,                                 /* 18 */
  SX127X_STATE_RX_RAW_PENDING_STOP,                               
  SX127X_STATE_RX_RAW_STOP,
  SX127X_STATE_TX_RAW,
  SX127X_STATE_TX_RAW_DONE,
#endif
};

struct sx127x_freq_coeff_s
{
  uint64_t freq;
  uint64_t chan;
};

struct sx127x_config_s
{
  /* freq */
  uint8_t freq[3];
  /* Others parameters */
  uint8_t cfg[64];
  /* mode */
  uint8_t mode;
  /* sync config */
  uint8_t sync;
  /* rx config */
  uint8_t rxcfg;
  /* Channel */
  uint8_t channel;
  /* Coefficients to improve frequency computation */
  struct sx127x_freq_coeff_s coeff;
  /* rssi_th */
  dev_rfpacket_pwr_t rssi_th;
};

#include <arch/efm32/timer.h>
#include <device/clock.h>

struct sx127x_private_s
{
  dev_timer_value_t timeout;
  dev_timer_value_t deadline;
  dev_timer_value_t timestamp;

  struct device_s                    *dev;
  /* base 5 ms time */
  dev_timer_delay_t                  delay_1ms;
  /* Bit time in us */
  dev_timer_delay_t                  timebit;
  /* Time before RX stop in us */
  dev_timer_delay_t                  tbrs;
  /* Time to achieve rx detection */
  dev_timer_delay_t                  tpbrx;

  /* Interrupt count */
  uint8_t                            icount;
  uint8_t                            cancel;
  /* Size of last received packet */
  uint8_t                            size;
  /* Internal state */
  enum sx127x_state_e                state;
  /* SPI controller */
  struct device_spi_ctrl_s           spi;
  struct dev_spi_ctrl_bytecode_rq_s  spi_rq;
  /* SX127x gpio mapping */
  gpio_id_t                          pin_map[SX127X_PIN_COUNT];
  /* SX127x interrupts. */
  struct dev_irq_src_s               src_ep;
  /* TX/RX request queue */
  dev_request_queue_root_t           queue;
  /* RX continuous request */
  struct dev_rfpacket_rq_s *         rx_cont;
  struct dev_rfpacket_rq_s *         next_rx_cont;
  struct dev_rfpacket_rx_s *         rxrq;   

  struct kroutine_s                  kr;
  bool_t                             bcrun;
  uint32_t                           bc_status;

#ifdef SX127X_PRINT_REGS
  uint8_t                            *dump;
#endif
  struct sx127x_config_s             cfg;

  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;

  /* SPI timer */
  struct device_timer_s *            timer;
  struct dev_timer_rq_s              trq;

  struct device_bitbang_s            bitbang;
  struct dev_bitbang_rq_s            brq;
};

STRUCT_COMPOSE(sx127x_private_s, kr);
STRUCT_COMPOSE(sx127x_private_s, trq);
STRUCT_COMPOSE(sx127x_private_s, brq);

DRIVER_PV(struct sx127x_private_s);



























