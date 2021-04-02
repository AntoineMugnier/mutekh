#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>
#include <stdbool.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>
#include <device/class/rfpacket.h>
#include <device/class/crypto.h>

#include "sx1276_regs_fsk.h"

#define SX127X_PIN_COUNT 3
#define SX127X_IO_RST  0
#define SX127X_DIO0    1
#define SX127X_DIO4    2

#define STATUS          15
#define R_CTX_PV        14

#define CMD_FREQ_CHANGE      0x10000
#define CMD_RX_SCANNING      0x20000
#define CMD_RXC_RAW          0x40000
#define CMD_IO_MODE          0x80000

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

// Fifo values
#define SX127X_FIFO_SIZE 64

// lora
struct sx127x_lora_config_s
{
  union
    {
      struct
        {
          /* Modem configuration */
          uint8_t modemcfg[3];
          /* Sync word value */
          uint8_t sw;
          /* Preamble length */
          uint8_t pl;
          /* Inverted I/Q */
          uint8_t iq;

          uint8_t __padding[2];
        };
      uint8_t data[8];
    };
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

struct sx127x_private_s
{

  // Generic rfpacket context struct
  struct dev_rfpacket_ctx_s gctx;

  /* base 1 ms time */
  dev_timer_delay_t                  delay_1ms;
  /* Bit time in us */
  dev_timer_delay_t                  timebit;
  /* Time before RX stop in us */
  dev_timer_delay_t                  tbrs;
  /* Time to achieve rx detection */
  dev_timer_delay_t                  tpbrx;

  uint16_t                           osc_ppb;

  /* Configuration register */
  struct sx127x_config_s             cfg_regs;
  uint8_t                            cfg_offset;

  /* Packet infos from bytecode (rssi, snr) */
  uintptr_t                          bc_pkt_infos;

  /* SPI controller */
  struct device_spi_ctrl_s           spi;
  struct dev_spi_ctrl_bytecode_rq_s  spi_rq;

  /* SPI timer */
  struct device_timer_s *            timer;

  /* SX127x gpio mapping */
  gpio_id_t                          pin_map[3];

  /* SX127x interrupts. */
  struct dev_irq_src_s               src_ep[2];

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
  /* RNG crypto request */
  struct dev_crypto_rq_s *           crypto_rq;
#endif

  struct device_s *dev;

  // Driver flags/status
  uint8_t flags;
  uint8_t bc_status;
  // Config structs
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;
};

DRIVER_PV(struct sx127x_private_s);

// Config functions
error_t sx127x_build_config(struct sx127x_private_s * pv);
bool sx127x_config_check_fairtx_valid(const struct dev_rfpacket_rf_cfg_s *rfcfg);
void sx127x_config_freq(struct sx127x_private_s *pv, uint32_t channel);

// Flags
#define SX127X_FLAGS_RX_CONTINOUS 0x01 // rx continous is active
#define SX127X_FLAGS_RXC_INFINITE 0x02 // rx continous has no lifetime
#define SX127X_FLAGS_RX_TX_OK     0x04 // rx during tx_lbt is possible
#define SX127X_FLAGS_TX_POWER_OK  0x08 // tx power don't need to be configured
#define SX127X_FLAGS_RF_CONFIG_OK 0x10 // rf config don't need to be configured
#define SX127X_FLAGS_PK_CONFIG_OK 0x20 // packet config don't need to be configured
#define SX127X_FLAGS_TX_LBT       0x40 // current tx request is tx_lbt

// Byte code status values (dev_rfpacket_status_s mirror)
#define SX127X_BC_STATUS_RX_DONE 0
#define SX127X_BC_STATUS_TX_DONE 1
#define SX127X_BC_STATUS_RX_TIMEOUT 2
#define SX127X_BC_STATUS_TX_TIMEOUT 3
#define SX127X_BC_STATUS_CRC_ERR 4
#define SX127X_BC_STATUS_JAMMING_ERR 5
#define SX127X_BC_STATUS_OTHER_ERR 6
#define SX127X_BC_STATUS_MISC 7




#define SX127X_MODE_SLEEP	                      0x00
#define SX127X_MODE_STANDBY	                    0x01
#define SX127X_MODE_TX                          0x03
#define SX127X_MODE_RX_CONT                     0x05
#define SX127X_MODE_RX                          0x06
#define SX127X_MODE_LORA                        0x80

#define SX127X_IRQ_LORA_RX_TIMEOUT              0x80
#define SX127X_IRQ_LORA_RX_DONE                 0x40
#define SX127X_IRQ_LORA_CRC_ERROR               0x20
#define SX127X_IRQ_LORA_VALID_HEADER            0x10
#define SX127X_IRQ_LORA_TX_DONE                 0x08
#define SX127X_IRQ_LORA_CAD_DONE                0x04
#define SX127X_IRQ_LORA_FHSS_CHANGE             0x02
#define SX127X_IRQ_LORA_CAD_DETECTED            0x01

#define SX127X_IRQ_LORA_DIS_IRQMASK             0xff
#define SX127X_IRQ_LORA_RX_IRQMASK              0xbf
#define SX127X_IRQ_LORA_RX_CONT_IRQMASK         0xbf
#define SX127X_IRQ_LORA_TX_IRQMASK              0xf7

#define RLR_FIFO                                0x00
#define RLR_OPMODE                              0x01
#define RLR_FRFMSB                              0x06
#define RLR_FRFMID                              0x07
#define RLR_FRFLSB                              0x08
#define RLR_PACONFIG                            0x09
#define RLR_PARAMP                              0x0A
#define RLR_OCP                                 0x0B
#define RLR_LNA                                 0x0C
#define RLR_FIFOADDRPTR                         0x0D
#define RLR_FIFOTXBASEADDR                      0x0E
#define RLR_FIFORXBASEADDR                      0x0F
#define RLR_FIFORXCURRENTADDR                   0x10
#define RLR_IRQFLAGSMASK                        0x11
#define RLR_IRQFLAGS                            0x12
#define RLR_RXNBBYTES                           0x13
#define RLR_RXHEADERCNTVALUEMSB                 0x14
#define RLR_RXHEADERCNTVALUELSB                 0x15
#define RLR_RXPACKETCNTVALUEMSB                 0x16
#define RLR_RXPACKETCNTVALUELSB                 0x17
#define RLR_MODEMSTAT                           0x18
#define RLR_PKTSNRVALUE                         0x19
#define RLR_PKTRSSIVALUE                        0x1A
#define RLR_RSSIVALUE                           0x1B
#define RLR_HOPCHANNEL                          0x1C
#define RLR_MODEMCONFIG1                        0x1D
#define RLR_MODEMCONFIG2                        0x1E
#define RLR_SYMBTIMEOUTLSB                      0x1F
#define RLR_PREAMBLEMSB                         0x20
#define RLR_PREAMBLELSB                         0x21
#define RLR_PAYLOADLENGTH                       0x22
#define RLR_PAYLOADMAXLENGTH                    0x23
#define RLR_HOPPERIOD                           0x24
#define RLR_FIFORXBYTEADDR                      0x25
#define RLR_MODEMCONFIG3                        0x26
#define RLR_FEIMSB                              0x28
#define RLR_FEIMID                              0x29
#define RLR_FEILSB                              0x2A
#define RLR_RSSIWIDEBAND                        0x2C
#define RLR_DETECTOPTIMIZE                      0x31
#define RLR_INVERTIQ                            0x33
#define RLR_DETECTIONTHRESHOLD                  0x37
#define RLR_SYNCWORD                            0x39
#define RLR_DIOMAPPING1                         0x40
#define RLR_DIOMAPPING2                         0x41
#define RLR_VERSION                             0x42
#define RLR_PLLHOP                              0x44
#define RLR_TCXO                                0x4B
#define RLR_PADAC                               0x4D
#define RLR_FORMERTEMP                          0x5B
#define RLR_BITRATEFRAC                         0x5D
#define RLR_AGCREF                              0x61
#define RLR_AGCTHRESH1                          0x62
#define RLR_AGCTHRESH2                          0x63
#define RLR_AGCTHRESH3                          0x64
#define RLR_PLL                                 0x70
