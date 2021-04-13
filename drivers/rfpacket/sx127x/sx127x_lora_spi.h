#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>
#include <device/class/rfpacket.h>
#include <device/class/crypto.h>

#define SX127X_IO_RST  0
#define SX127X_IO_DIO0 1
#define SX127X_IO_DIO3 2

#define R_TMP0 0
#define R_TMP1 1
#define R_TMP2 2
#define R_TMP3 3
#define R_TMP4 4
#define R_TMP5 5
#define R_TMP6 6
#define R_ARG0 7
#define R_ARG1 8

#define R_CTX_PV   10
#define R_LINK	   12
#define R_ICOUNT   13

#define SX127X_INIT_MASK        (1 << 0)
#define SX127X_TX_MASK	        (1 << 1)
#define SX127X_RX_VALID_MASK	(1 << 2)
#define SX127X_RX_MASK	        (1 << 3)
#define SX127X_RX_PACKET_MASK   (1 << 4)
#define SX127X_CFG_MASK	        (1 << 5)
#define SX127X_CANCEL_MASK      (1 << 6)
#define SX127X_RX_TIMEOUT_MASK  (1 << 7)

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
# define SX127X_RNG_MASK        (1 << 0)
#endif

#ifndef __MUTEK_ASM__

struct sx127x_bytecode_entry_s;
extern struct sx127x_bytecode_entry_s sx127x_entry_reset;
extern struct sx127x_bytecode_entry_s sx127x_entry_config;
extern struct sx127x_bytecode_entry_s sx127x_entry_tx;
extern struct sx127x_bytecode_entry_s sx127x_entry_rx;
extern struct sx127x_bytecode_entry_s sx127x_entry_rx_cont;
extern struct sx127x_bytecode_entry_s sx127x_entry_rx_packet;
extern struct sx127x_bytecode_entry_s sx127x_entry_cancel;
extern struct sx127x_bytecode_entry_s sx127x_entry_irq;

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
extern struct sx127x_bytecode_entry_s sx127x_entry_rng;
#endif

extern const struct bc_descriptor_s sx127x_bytecode;

#endif

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0

struct sx127x_pk_cfg_cache_entry_s
{
  /* The cached LoRa packet configuration */
  struct dev_rfpacket_pk_cfg_lora_s const * cfg;

  /* Precomputed register values */
  uint8_t                                   modemcfg[2];
};

struct sx127x_rf_cfg_cache_entry_s
{
  /* The cached LoRa RF configuration */
  struct dev_rfpacket_rf_cfg_lora_s const * cfg;

  /* Precomputed register values */
  uint8_t                                   modemcfg[3];
};

struct sx127x_freq_cache_entry_s
{
  /* The channel id (key of cache entry) */
  int16_t channel;

  /* The Frf register pre-computed value */
  uint8_t data[4];
};

enum sx127x_config_msk_e
{
  SX127X_CFG_CACHE_MSK_PK = 0x1,
  SX127X_CFG_CACHE_MSK_RF = 0x2,
  SX127X_CFG_CACHE_MSK_FQ = 0x4,

  SX127X_CFG_CACHE_MSK_ALL = ( SX127X_CFG_CACHE_MSK_PK
                             | SX127X_CFG_CACHE_MSK_RF
                             | SX127X_CFG_CACHE_MSK_FQ),
};

#endif // CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0

enum sx127x_state_e
{
  SX127X_STATE_IDLE,
  SX127X_STATE_SLEEP,
  SX127X_STATE_CONFIG,
  SX127X_STATE_RX,
  SX127X_STATE_RX_PAYLOAD,
  SX127X_STATE_RX_CONTINUOUS,
  SX127X_STATE_TX,
};

struct sx127x_config_s
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

struct sx127x_private_s
{
  /* base 5 ms time */
  dev_timer_delay_t                  delay_5ms;

  /* Queue head */
  struct dev_rfpacket_rq_s *         next_rq;

  /* Current RX buffer */
  struct dev_rfpacket_rx_s *         rx_last;

  /* Done flags */
  uint16_t                           done;

  /* Interrupt count */
  uint16_t                           icount;

  /* Configuration register backup. */
  struct sx127x_config_s             bkp_config;
  uint8_t                            bkp_freq[4];
  /* Configuraton mask (i.e. what to send over spi). */
  uint8_t                            bkp_config_mask;

  /* Packet infos from bytecode (rssi, snr) */
  uintptr_t                          bc_pkt_infos;

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
  /* Configuration cache */
  uint8_t                            dirty;
  struct sx127x_pk_cfg_cache_entry_s pk_cfg_cache[CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE];
  struct sx127x_rf_cfg_cache_entry_s rf_cfg_cache[CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE];
  struct sx127x_freq_cache_entry_s   freq_cache[CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE];
#endif

  /* Internal state */
  enum sx127x_state_e                state;

  /* Identifier of last request configuration */
  uint32_t                           last_cfg_id;

  /* SPI controller */
  struct device_spi_ctrl_s           spi;
  struct dev_spi_ctrl_bytecode_rq_s  spi_rq;

  /* SPI timer */
  struct device_timer_s *            timer;

  /* Timestamp holder */
  dev_timer_value_t                  timestamp;

  /* SX127x gpio mapping */
  gpio_id_t                          pin_map[3];

  /* SX127x interrupts. */
  struct dev_irq_src_s               src_ep[2];

  /* TX/RX request queue */
  dev_request_queue_root_t           queue;
  /* RX continuous request */
  struct dev_rfpacket_rq_s *         rx_cont_rq;

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_STATS)
  struct dev_rfpacket_stats_s        stats;
#endif

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
  /* RNG crypto request */
  struct dev_crypto_rq_s *           crypto_rq;
#endif
};

DRIVER_PV(struct sx127x_private_s);

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
