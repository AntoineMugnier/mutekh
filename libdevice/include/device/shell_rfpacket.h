#ifndef SHELL_RFPACKET_H_
# define SHELL_RFPACKET_H_

#include <mutek/console.h>
#include <device/shell.h>

enum rfpacket_opts_e
{
  RFPACKET_OPT_DEV       = 0x01,
  RFPACKET_OPT_LIFETIME  = 0x02,
  RFPACKET_OPT_DATA      = 0x04,
  RFPACKET_OPT_PWR       = 0x08,
  RFPACKET_OPT_SIZE      = 0x10,
  RFPACKET_OPT_FREQ      = 0x20,
  RFPACKET_OPT_DEVIATION = 0x40,
  RFPACKET_OPT_BW        = 0x80,
  RFPACKET_OPT_DRATE     = 0x100,
  RFPACKET_OPT_MOD       = 0x200,
  RFPACKET_OPT_SYMB      = 0x400,
  RFPACKET_OPT_CFG       = 0x800,
  RFPACKET_OPT_ENCODING  = 0x1000,
  RFPACKET_OPT_CRC       = 0x2000,
  RFPACKET_OPT_SW_VAL    = 0x40000,
  RFPACKET_OPT_SW_LEN    = 0x80000,
  RFPACKET_OPT_PB_VAL    = 0x10000,
  RFPACKET_OPT_PB_LEN    = 0x20000,
  RFPACKET_OPT_RX_PB_LEN = 0x40000,
  RFPACKET_OPT_TX_PB_LEN = 0x800000,
};

#define RFPACKET_OPT_RF_CFG_MSK (RFPACKET_OPT_FREQ | \
                                 RFPACKET_OPT_DEVIATION | \
                                 RFPACKET_OPT_BW | \
                                 RFPACKET_OPT_DRATE | \
                                 RFPACKET_OPT_MOD | \
                                 RFPACKET_OPT_SYMB)


#define RFPACKET_OPT_PKT_CFG_MSK (RFPACKET_OPT_CRC | \
                                  RFPACKET_OPT_ENCODING |\
                                  RFPACKET_OPT_SW_VAL |\
                                  RFPACKET_OPT_SW_LEN |\
                                  RFPACKET_OPT_PB_VAL |\
                                  RFPACKET_OPT_PB_LEN |\
                                  RFPACKET_OPT_RX_PB_LEN |\
                                  RFPACKET_OPT_TX_PB_LEN)

TERMUI_CON_COMMAND_PROTOTYPE(shell_rfpacket_configure);

struct termui_optctx_dev_rfpacket_opts
{
  struct device_rfpacket_s accessor;

  union 
    {
      /* TX */
      struct 
        { 
          uint32_t lifetime;
          size_t size;
          struct shell_opt_buffer_s cfg;
          struct shell_opt_buffer_s data;
          int16_t pwr;
        };
 
      /* CFG */
      struct
        {
          /* RF configuration */
          enum dev_rfpacket_modulation_e  mod;
          uint8_t symbols;
          uint32_t frequency;
          uint32_t deviation;
          uint32_t bw;
          uint32_t drate;

          /* Packet configuration */
          uint8_t                       encoding;
          uint32_t                      crc;
          uint32_t                      sw_value;
          uint32_t                      pb_pattern;
          uint8_t                       sw_len;
          uint8_t                       pb_pattern_len;
          uint8_t                       tx_pb_len;
          uint8_t                       rx_pb_len;
        };
    };
};

#endif /* !SHELL_RFPACKET_H_ */
