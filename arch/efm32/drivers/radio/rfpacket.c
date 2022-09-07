/*
   This file is part of MutekH.

   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as
   published by the Free Software Foundation; version 2.1 of the
   License.

   MutekH is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.

   Copyright (c) 2017 Sebastien Cerdan <sebcerdan@gmail.com>

 */

#include "common.h"
#include "protimer.h"
#include <device/class/iomux.h>

/* LBT parameters */
#define EFR32_ETSI_LBT_TIME              5000ULL     /*us*/
#define EFR32_MAX_LBT_RETRY_COUNT        4

/* Protimer Radio class parameters */  
#define EFR32_PROTIMER_RX_START_CHANNEL  1
#define EFR32_PROTIMER_RX_STOP_CHANNEL   2
#define EFR32_PROTIMER_TX_CHANNEL        3

#define EFR32_RADIO_THRESHOLD 16

#define EFR32_RADIO_RFP_BUFFER_SIZE 256

#define EFR32_RX_IRQ_FRC_MSK      (EFR32_FRC_IF_RXDONE     |      \
                                   EFR32_FRC_IF_RXOF       |      \
                                   EFR32_FRC_IF_BLOCKERROR |      \
                                   EFR32_FRC_IF_FRAMEERROR)

enum efr32_radio_rfp_state
{
  EFR32_RFP_STATE_IDLE,
  EFR32_RFP_STATE_WAIT_FOR_IDLE,
  EFR32_RFP_STATE_RX,
  EFR32_RFP_STATE_RX_PENDING_RX,
  EFR32_RFP_STATE_RX_DONE_PENDING_RX,
  EFR32_RFP_STATE_RXC,                             /* 5 */
  EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX,
  EFR32_RFP_STATE_RXC_PENDING_RX,
  EFR32_RFP_STATE_TX,
  EFR32_RFP_STATE_TX_LBT,                          /* 9 */
  EFR32_RFP_STATE_TX_LBT_RX_ONGOING
};

DRIVER_PV(struct radio_efr32_rfp_ctx_s
{
  struct radio_efr32_ctx_s      pv;

  dev_timer_value_t             deadline;
  dev_request_queue_root_t      queue;
  struct dev_rfpacket_rq_s      *rx_cont;
  struct dev_rfpacket_rq_s      *next_rx_cont;
  struct dev_rfpacket_rx_s      *rx;
  uint8_t                       sg_buffer[EFR32_RADIO_RFP_BUFFER_SIZE];
  bool_t 		        sleep;
  enum efr32_radio_rfp_state    state;
  struct device_cmu_s           clock;
  /* Current config */
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;
});

STRUCT_COMPOSE(radio_efr32_rfp_ctx_s, pv);

static void efr32_rfp_idle(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_timer_irq(struct device_s *dev);
static void efr32_rfp_end_rq(struct radio_efr32_rfp_ctx_s *ctx, error_t err);
static error_t efr32_rfp_fsk_init(struct radio_efr32_rfp_ctx_s *ctx);
static error_t efr32_rfp_sigfox_init(struct radio_efr32_rfp_ctx_s *ctx);

static void efr32_radio_set_state(struct radio_efr32_rfp_ctx_s *pv, enum efr32_radio_rfp_state state)
{
  efr32_radio_printk("drv: st %d\n", state);
  pv->state = state;
}


static void efr32_rfp_schedule_next_wup(struct radio_efr32_rfp_ctx_s *ctx)
{
  uint32_t x = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CCV_ADDR(1), 10000 + x);
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CCV_ADDR(2), 10000 + 5000 + x);
#ifdef CONFIG_DEVICE_CLOCK_GATING
  /* Stop all clock exept for RTCC */
  for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT - 1; i++)
    dev_clock_sink_gate(&ctx->pv.clk_ep[i], DEV_CLOCK_EP_POWER);
  ctx->sleep = 1;
#endif
}

static void efr32_rfp_start_rx_ldc(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CTRL_ADDR(1), EFM32_RTCC_CC_CTRL_MODE_SHIFT_VAL(OUTPUTCOMPARE));
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CC_CTRL_ADDR(2), EFM32_RTCC_CC_CTRL_MODE_SHIFT_VAL(OUTPUTCOMPARE));
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, endian_le32(EFM32_RTCC_IFC_MASK));
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR, EFM32_RTCC_IEN_CC(2) | EFM32_RTCC_IEN_CC(1));
  efr32_rfp_schedule_next_wup(ctx);
  /* Start Counter */ 
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, endian_le32(EFM32_RTCC_CTRL_ENABLE));
}


static inline error_t efr32_rf_config(struct radio_efr32_rfp_ctx_s *ctx,
                                      struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

  if ((rfcfg != ctx->rf_cfg) || rfcfg->cache.dirty)
  /* Test if new RF configuration or previous configuration modified */
    {
      if (rfcfg->mod == DEV_RFPACKET_GFSK)
        efr32_rfp_fsk_init(ctx);
      else if (rfcfg->mod == DEV_RFPACKET_SIGFOX)
        efr32_rfp_sigfox_init(ctx);
      else
        return -ENOTSUP;

      ctx->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)rfcfg;

      uint32_t div = cpu_mem_read_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR);
      div = EFR32_SYNTH_DIVCTRL_LODIVFREQCTRL_GET(div);

      /* Configure frequency */
      uint64_t f = ((uint64_t)(rfcfg->frequency) * div) << 19;
      f /= EFR32_RADIO_HFXO_CLK;
      cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_FREQ_ADDR, (uint32_t)f);

      /* Configure channel spacing */
      uint64_t chsp = ((uint64_t)(rfcfg->chan_spacing) * div) << 19;
      chsp /= EFR32_RADIO_HFXO_CLK;
      cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHSP_ADDR, (uint32_t)chsp);

    }

  return 0;
}

static inline error_t efr32_pkt_config(struct radio_efr32_rfp_ctx_s *ctx,
                                       struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;

  /* Test if new Packet configuration or previous configuration modified */
  if ((pkcfg == ctx->pk_cfg) && !pkcfg->cache.dirty)
    return 0;

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_SIGFOX)
    {
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DATABUFFER_ADDR, 0x6f);
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEPOLY_ADDR, 0x80);
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEINIT_ADDR, 0xff);
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FECCTRL_ADDR, 0x2);
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(0), 0xff);
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(2), 0x1ff);

      /* No additionnal length byte */
      uint32_t x = EFR32_FRC_CTRL_RXFCDMODE(FCDMODE0) |
                   EFR32_FRC_CTRL_TXFCDMODE(FCDMODE0) |
                   EFR32_FRC_CTRL_BITORDER(MSB) |
                   EFR32_FRC_CTRL_BITSPERWORD(7);
  
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);
  
      x = EFR32_FRC_DFLCTRL_DFLMODE(DISABLE);
  
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, x);
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MAXLENGTH_ADDR, EFR32_RADIO_RFP_BUFFER_SIZE - 1);

      /* No preamble - No Sync word - No CRC */
      return 0;
    }

  if (rq->pk_cfg->format != DEV_RFPACKET_FMT_SLPC)
    return -ENOTSUP;

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_RXCTRL_ADDR, EFR32_FRC_RXCTRL_BUFRESTOREFRAMEERROR |
                                                           EFR32_FRC_RXCTRL_BUFRESTORERXABORTED);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_TRAILRXDATA_ADDR, EFR32_FRC_TRAILRXDATA_RSSI);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(0), EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(1), EFR32_FRC_FCD_WORDS(255) |
                                                           EFR32_FRC_FCD_INCLUDECRC |
                                                           EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(2), EFR32_FRC_FCD_BUFFER(1) |
                                                           EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(3), EFR32_FRC_FCD_WORDS(255) |
                                                           EFR32_FRC_FCD_BUFFER(1) |
                                                           EFR32_FRC_FCD_INCLUDECRC |
                                                           EFR32_FRC_FCD_CALCCRC |
                                                           EFR32_FRC_FCD_SKIPWHITE);
  /* Configure variable length mode */
  uint32_t x = EFR32_FRC_CTRL_RXFCDMODE(FCDMODE2) |
               EFR32_FRC_CTRL_TXFCDMODE(FCDMODE2) |
               EFR32_FRC_CTRL_BITORDER(MSB) |
               EFR32_FRC_CTRL_BITSPERWORD(7);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);

  x = EFR32_FRC_DFLCTRL_DFLMODE(SINGLEBYTE) |
      EFR32_FRC_DFLCTRL_MINLENGTH(1) |
      EFR32_FRC_DFLCTRL_DFLBITS(8);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, x);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MAXLENGTH_ADDR, EFR32_RADIO_RFP_BUFFER_SIZE - 1);

  ctx->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pkcfg;

  const struct dev_rfpacket_pk_cfg_basic_s *cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

  /** Configure Sync Word */

  uint8_t sw = cfg->sw_len + 1;

  if ((sw >> 5) || (sw % 4))
    return -ENOTSUP;
  
  x = EFR32_MODEM_CTRL1_SYNCBITS(cfg->sw_len);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL1_ADDR, x);

  x = cfg->sw_value;

  x = ((x & 0x55555555) << 1) | ((0xAAAAAAAA & x) >> 1);
  x = ((x & 0x33333333) << 2) | ((0xCCCCCCCC & x) >> 2);
  x = ((x & 0x0F0F0F0F) << 4) | ((0xF0F0F0F0 & x) >> 4);
  x = ((x & 0xFF00FF00) >> 8) | ((0x00FF00FF & x) << 8);
  x = ((x & 0xFFFF0000) >> 16) | ((0x0000FFFF & x) << 16);
  x = x >> (32 - sw);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SYNC0_ADDR, x);

  /** Configure Preamble */

  if ((cfg->pb_pattern_len + 1) >> 4)
    return -ENOTSUP;

  x = EFR32_MODEM_PRE_BASEBITS(cfg->pb_pattern_len);

  uint32_t msk = (1 << (cfg->pb_pattern_len + 1)) - 1;
  uint32_t preamble = cfg->pb_pattern & msk;

  if (preamble == (0xAAAAAAAA & msk))
    EFR32_MODEM_PRE_BASE_SET(x, 10); /* TYPE 1010 */
  else if (preamble == (0x55555555 & msk))
    EFR32_MODEM_PRE_BASE_SET(x, 01); /* TYPE 0101 */
  else
    return -ENOTSUP;

  EFR32_MODEM_PRE_TXBASES_SET(x, cfg->tx_pb_len/(cfg->pb_pattern_len + 1));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_PRE_ADDR, x);

  /** Configure CRC */

  x =  EFR32_CRC_CTRL_BITSPERWORD(7) |
       EFR32_CRC_CTRL_INPUTBITORDER(MSB) |
       EFR32_CRC_CTRL_BITREVERSE(LSB);


  uint32_t w;

  switch (cfg->crc)
    {
    case 0:
    case 0x07:
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH8;
      break;
    case 0x8005:
    case 0x1021:
    case 0x3d65: 
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH16;
      break;
    case 0x65b:  
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH24;
      break;
    case 0x814141ab:
    case 0x04c11db7:
      w = EFR32_CRC_CTRL_CRCWIDTH_CRCWIDTH32;
      break;
    default:
      return -ENOTSUP;
    }

  x |= (w << EFR32_CRC_CTRL_CRCWIDTH_IDX);

  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_CTRL_ADDR, x);

  /* Reverse bits */

  uint32_t v = cfg->crc;
  x = 0;

  for (uint8_t i = 0; i < ((w + 1) << 3); i++)
    {
      x <<= 1;
      x = v & 1 ? x | 1 : x;
      v >>= 1;
    }

  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_POLY_ADDR, x);
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_INIT_ADDR, EFR32_CRC_INIT_INIT(cfg->crc_seed));

  return 0;
}

static void efr32_rfp_set_cca_threshold(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_fsk_s * c = const_dev_rfpacket_rf_cfg_fsk_s_cast(rq->rf_cfg);

  int16_t r = c->fairtx.lbt.rssi >> 3;
  int8_t v = (r & 0x7F) | (r < 0 ? 0x80 : 0); 

  /* Saturate rssi threshold */
  if (r < -128)
    v = -128;

  if (r > 127)
    v = 127;

  uint32_t x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_CCATHRSH_SET(x, v & 0xFF);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);
}


static void efr32_rfp_start_tx_lbt(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq)
{
 
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TOUTCNTTOP_ADDR(0), EFR32_PROTIMER_TOUTCNTTOP_PCNTTOP(4));
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TOUTCOMP_ADDR(0), EFR32_PROTIMER_TOUTCOMP_PCNTCOMP(1));

  /* Time granularity is based on T0UF */

  uint32_t x = /* Time between 2 retries */
               EFR32_PROTIMER_LBTCTRL_FIXEDBACKOFF |
               EFR32_PROTIMER_LBTCTRL_STARTEXP(1) |
               EFR32_PROTIMER_LBTCTRL_MAXEXP(1) |
               /* Number of consecutive successfull CCA before generating LBT success */ 
               EFR32_PROTIMER_LBTCTRL_CCAREPEAT(1) |
               /* Time between 2 CCA measures */
               EFR32_PROTIMER_LBTCTRL_CCADELAY(1) |
               /* Number of CCA failure before generating LBTFAILURE */ 
               EFR32_PROTIMER_LBTCTRL_RETRYLIMIT(EFR32_MAX_LBT_RETRY_COUNT - 1);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_LBTCTRL_ADDR, x);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RANDOM_ADDR, 1);

#if EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 2 || \ /* xg12 */
  EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 4  /* xg14 */
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_LBTSTATE1_ADDR, EFR32_PROTIMER_LBTSTATE1_EXP(1));
#endif

  x = EFR32_PROTIMER_RXCTRL_RXSETEVENT(0, ALWAYS) |
      EFR32_PROTIMER_RXCTRL_RXCLREVENT(0, ALWAYS) |

      EFR32_PROTIMER_RXCTRL_RXSETEVENT(1, TOUT0MATCHLBT) |     
      EFR32_PROTIMER_RXCTRL_RXCLREVENT(1, CCAACK);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RXCTRL_ADDR, x);

  x = EFR32_PROTIMER_TXCTRL_TXSETEVENT(0, ALWAYS) |
      EFR32_PROTIMER_TXCTRL_TXSETEVENT(1, LBTSUCCESS);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TXCTRL_ADDR, x);

  uint32_t nbsym = (EFR32_ETSI_LBT_TIME * rq->rf_cfg->drate)/1000000;
  nbsym = bit_ctz(pow2_up(nbsym));

  assert(nbsym < 15);

  /* Configure AGC */
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR);
  EFR32_AGC_CTRL0_MODE_SET(x, CONT);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR, x);

  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_RSSIPERIOD_SET(x, nbsym);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);

  efr32_rfp_set_cca_threshold(ctx, rq);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, EFR32_PROTIMER_CMD_LBTSTART);
}

static void efr32_rfp_try_restart_tx(struct radio_efr32_rfp_ctx_s *ctx)
{
  dev_timer_value_t time = efr32_protimer_get_value(&ctx->pv.pti);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR); 

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&ctx->queue);

  assert(rq->type == DEV_RFPACKET_RQ_TX_FAIR);

  if (ctx->deadline <= time)
  /* Deadline reached */
    {
      efr32_rfp_end_rq(ctx, -ETIMEDOUT);
      return;
    }

  efr32_rfp_start_tx_lbt(ctx, rq);
}

static void efr32_rfp_start_tx(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFR32_SEQ_DEADLINE_ADDR, 0);

  struct dev_rfpacket_rq_s *rq;
  rq = dev_rfpacket_rq_head(&ctx->queue);

  assert(rq);

  /* Check RAC state */
  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));

  assert(EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF);

  struct radio_efr32_ctx_s *pv = &ctx->pv;
  dev_timer_value_t time = efr32_protimer_get_value(&pv->pti);
  dev_timer_value_t start = rq->deadline ? rq->deadline : time;

  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR); 
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR); 

  /* Write length when required */
  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_SIGFOX)
    cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WCNTCMP0_ADDR, rq->tx_size - 1);
  else
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), rq->tx_size);

  uint8_t *p = (uint8_t *)rq->tx_buf;

  /* Fill payload */
  for(uint16_t i = 0; i < rq->tx_size; i++)
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), (uint32_t)p[i]);

  /* Set channel */
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, rq->channel); 

  /* Enable TX */
  switch (rq->type)
    {
      case DEV_RFPACKET_RQ_TX:
        cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXEN);
        break;
      case DEV_RFPACKET_RQ_TX_FAIR:
        ctx->deadline = start + rq->lifetime;
        efr32_rfp_start_tx_lbt(ctx, rq);
        break;
      default:
        abort();
    }
}

static void efr32_rfp_disable(struct radio_efr32_rfp_ctx_s *ctx)
{
  /* Disable RX */
  uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
  x &= ~0xF;
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, 0);
  
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CMD_ADDR, EFR32_FRC_CMD_RXABORT);
  
  /* Disable TX */
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXDIS);

  /* Clear irq */
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR, 0);
}

static void efr32_rfp_start_rx_scheduled(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;
 
#if EFR32_PROTIMER_HW_WIDTH < 64
  assert(0);
#endif
  dev_timer_value_t time = efr32_protimer_get_value(&pv->pti);
  dev_timer_value_t start = rq->deadline ? rq->deadline : time;
  
  ctx->deadline = start + rq->lifetime;

  if (ctx->deadline < time)
  /* Deadline reached */
    return efr32_rfp_end_rq(ctx, 0);
 
  uint32_t x = EFR32_PROTIMER_RXCTRL_RXSETEVENT(0, ALWAYS) |
               EFR32_PROTIMER_RXCTRL_RXCLREVENT(0, ALWAYS) |
               EFR32_PROTIMER_RXCTRL_RXSETEVENT(1, CC1)    |     
               EFR32_PROTIMER_RXCTRL_RXCLREVENT(1, CC2);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RXCTRL_ADDR, x);

  /* RX scheduled looks not to work as expected if no minimal delay */
  if ((start <= time) || (start - time) < 128)
    start = time + 128;

  efr32_protimer_request_start(&pv->pti, time, start, EFR32_PROTIMER_RX_START_CHANNEL);
  efr32_protimer_request_start(&pv->pti, time, ctx->deadline, EFR32_PROTIMER_RX_STOP_CHANNEL);
}

static void efr32_rfp_start_rx(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq)
{
  uint32_t x;
  cpu_mem_write_32(EFR32_SEQ_DEADLINE_ADDR, 0);

  /* Configure AGC */ 
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR);
  EFR32_AGC_CTRL1_RSSIPERIOD_SET(x, 5);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, x);
  x = cpu_mem_read_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR);
  EFR32_AGC_CTRL0_MODE_SET(x, LOCKPREDET);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR, x);

  /* Check RAC state */
  x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  assert(EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF);

  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR); 

  /* Set channel */
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, rq->channel); 

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_RX_CONT:
      assert(ctx->rx_cont);
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC);
      /* Enable RX */
     // efr32_rfp_ldc_prs(ctx);
      efr32_rfp_start_rx_ldc(ctx);
      break;
    case DEV_RFPACKET_RQ_RX:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX);
      /* Scheduled RX */
      efr32_rfp_start_rx_scheduled(ctx, rq);
      break;
    default:
      assert(1);
  }
}


static inline void efr32_rfp_process_group(struct radio_efr32_rfp_ctx_s *ctx, bool_t group)
{
  struct dev_rfpacket_rq_s * rq;

  while (1)
  {
    rq = dev_rfpacket_rq_head(&ctx->queue);

    if (!rq || rq->err_group != group)
      break;

    rq->error = -ECANCELED;
    dev_rfpacket_rq_pop(&ctx->queue);
    dev_rfpacket_rq_done(rq);
  }
}

static void efr32_rfp_test_timeout(struct radio_efr32_rfp_ctx_s *ctx)
{
  efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX);

  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR); 

  dev_timer_value_t time = efr32_protimer_get_value(&ctx->pv.pti);

  if (ctx->deadline <= time)
    {
      printk("endrq\n");
      efr32_rfp_end_rq(ctx, 0);
    }
}

static void efr32_rfp_read_done(struct radio_efr32_rfp_ctx_s *ctx)
{
  switch (ctx->state)
    {
      case EFR32_RFP_STATE_RX_DONE_PENDING_RX:
        efr32_rfp_idle(ctx);
        break;
      case EFR32_RFP_STATE_RX_PENDING_RX:
      case EFR32_RFP_STATE_RX:
        efr32_rfp_test_timeout(ctx);
        break;
      case EFR32_RFP_STATE_RXC:
      case EFR32_RFP_STATE_RXC_PENDING_RX:
        assert(ctx->rx_cont);
        efr32_rfp_idle(ctx);
        break;
      case EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX:
        /* Radio is disabled here */
        assert(ctx->rx_cont);
        if (ctx->next_rx_cont != ctx->rx_cont)
          {
            dev_rfpacket_rq_done(ctx->rx_cont);
            ctx->rx_cont = ctx->next_rx_cont;
          }
        ctx->next_rx_cont = NULL;
        efr32_rfp_idle(ctx);
        break;
      default:
        abort();
    }
}

static int16_t efr32_rfp_get_rssi(struct radio_efr32_rfp_ctx_s *ctx, uintptr_t a)
{
  int16_t x = cpu_mem_read_32(EFR32_AGC_ADDR + a);
  x >>= EFR32_AGC_RSSI_RSSIFRAC_IDX;
  
  if ((x >> 9) & 1)
  /* Negative value */
    {
      uint16_t msk = EFR32_AGC_RSSI_MASK >> EFR32_AGC_RSSI_RSSIFRAC_IDX;
      x |= 0xFFFF ^ msk;
      x -= 1;
      x = -(~x & msk);
    }

  return  (x << 3) >> 2;
}

static void efr32_rfp_read_packet(struct radio_efr32_rfp_ctx_s *ctx, bool_t locked)
{
  struct dev_rfpacket_rx_s *rx = ctx->rx;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  rx->timestamp = 0;
  rx->snr = 0;
  rx->error = 0;
  rx->carrier = 0;
 
  uint8_t *p = (uint8_t *)rx->buf;

  /* Read packet */
  for (uint16_t i = 0; i < rx->size; i++)
    p[i] = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));

  rx->rssi = efr32_rfp_get_rssi(ctx, EFR32_AGC_FRAMERSSI_ADDR);

  kroutine_exec(&rx->kr);

  if (locked)
    return efr32_rfp_read_done(ctx);
  
  LOCK_SPIN_IRQ(&pv->dev->lock);
  efr32_rfp_read_done(ctx);
  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static KROUTINE_EXEC(efr32_rfp_tx)
{
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_rfp_start_tx(ctx);
}

static KROUTINE_EXEC(efr32_rfp_rx)
{
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_rfp_read_packet(ctx, 0);
}
static KROUTINE_EXEC(efr32_rfp_ldc)
{
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);
  struct radio_efr32_rfp_ctx_s *ctx = radio_efr32_rfp_ctx_s_from_pv(pv);

  efr32_rfp_schedule_next_wup(ctx);
}

static void efr32_rfp_end_rq(struct radio_efr32_rfp_ctx_s *ctx, error_t err)
{
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&ctx->queue);

  assert(rq && rq->type != DEV_RFPACKET_RQ_RX_CONT);

  dev_rfpacket_rq_pop(&ctx->queue);
  dev_rfpacket_rq_done(rq);

  rq->error = err;

  if (rq->error)
    efr32_rfp_process_group(ctx, rq->err_group);

  /* Timeout has occured before we fill RX fifo */
  if (ctx->state == EFR32_RFP_STATE_RX_PENDING_RX)
    return efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX_DONE_PENDING_RX);

  return efr32_rfp_idle(ctx);
}

static void efr32_rfp_rx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct dev_rfpacket_rq_s *rq = NULL;

  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  x = EFR32_RAC_STATUS_STATE_GET(x);  
 
   switch (ctx->state)
   {
     case EFR32_RFP_STATE_TX_LBT_RX_ONGOING:
       /* Flush RX fifo */
       cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
       /* TX LBT is finished */
       return efr32_rfp_try_restart_tx(ctx);
     case EFR32_RFP_STATE_WAIT_FOR_IDLE:
     case EFR32_RFP_STATE_TX_LBT:
       /* Flush RX fifo */
       cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
       return;  
     case EFR32_RFP_STATE_RXC:
       if (irq != EFR32_FRC_IF_RXDONE)
         {
           /* Flush RX fifo */
           cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
           return efr32_rfp_idle(ctx);
         }
       rq = ctx->rx_cont;
       break;
     case EFR32_RFP_STATE_RX:
       if (irq != EFR32_FRC_IF_RXDONE)
         {
           /* Flush RX fifo */
           cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
           return;
         }
       rq = dev_rfpacket_rq_head(&ctx->queue);
       break;
     default:
       abort();
   }

  assert(rq);

  uint16_t size = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);

  uint32_t status = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_STATUS_ADDR(1));
  status = EFR32_BUFC_STATUS_BYTES_GET(status);

  struct dev_rfpacket_rx_s * rx = NULL;

  if (status >= size)
    rx = rq->rx_alloc(rq, size);
  else
    printk("mismatch %d %d\n", status, size);
  
  if (rx == NULL)
  /* Flush RX fifo */
    {
      cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
      cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);

      switch (ctx->state)
      {
        case EFR32_RFP_STATE_RXC:
          return efr32_rfp_idle(ctx);
        case EFR32_RFP_STATE_RX:
          return efr32_rfp_test_timeout(ctx);
        default:
          abort();
      }
    }

  assert(rx->size == size);
  ctx->rx = rx;
  rx->channel = rq->channel;
  
  if (size <= EFR32_RADIO_THRESHOLD)
    return efr32_rfp_read_packet(ctx, 1);

  switch (ctx->state)
  {
    case EFR32_RFP_STATE_RXC:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC_PENDING_RX);
      break;
    case EFR32_RFP_STATE_RX:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX_PENDING_RX);
      break;
    default:
      abort();
  }

  kroutine_init_deferred(&pv->kr, &efr32_rfp_rx);
  kroutine_exec(&pv->kr);
}

static inline void efr32_rfp_tx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&ctx->queue);
  assert(rq);

  switch (ctx->state)
  {
    case EFR32_RFP_STATE_TX_LBT:
    case EFR32_RFP_STATE_TX:
      break;
    default:
      abort();
  }

  if (irq & EFR32_FRC_IF_TXDONE)
  /* Packet sent */
    {
      /* Set timestamp */
      rq->error = 0;
      rq->tx_timestamp = 0;
    }
  else
    /* Clear buffer */
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR);

  return efr32_rfp_end_rq(ctx, 0);
}

static bool_t efr32_check_rac_off(struct radio_efr32_rfp_ctx_s *ctx)
{
  /* Check RAC state */
  uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
  if (EFR32_RAC_STATUS_STATE_GET(x) != EFR32_RAC_STATUS_STATE_OFF)
    {
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFC_ADDR, EFR32_RAC_IF_STATECHANGE);
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IEN_ADDR, EFR32_RAC_IF_STATECHANGE);
      x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
      /* Trigger IRQ in case we miss the transition */
      if (EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF)
        cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFS_ADDR, EFR32_RAC_IF_STATECHANGE);
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_WAIT_FOR_IDLE);
      return 0;
    }
  efr32_radio_set_state(ctx, EFR32_RFP_STATE_IDLE);
  return 1;
}

static DEV_IRQ_SRC_PROCESS(efr32_radio_irq)
{
  struct device_s *dev = ep->base.dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;

  lock_spin(&dev->lock);

  uint32_t irq = 0;
  uint32_t x;
  uint32_t idx = ep - ctx->pv.irq_ep;

  switch (idx)
  {
    case 0:
      irq = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IEN_ADDR);
      cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_IFC_ADDR, irq);
      printk("modem irq: 0x%x\n", irq);
      break;
    case 1:
    case 2:
      assert(ctx->state == EFR32_RFP_STATE_WAIT_FOR_IDLE);
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IEN_ADDR, 0);
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFC_ADDR, EFR32_RAC_IF_STATECHANGE);
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_IDLE);
      /* Chack that Rac is idle */
      if (efr32_check_rac_off(ctx))
        efr32_rfp_idle(ctx);
      break;
    case 3:
      irq = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IEN_ADDR);
      printk("bufc irq: 0x%x\n", irq);
      break;
    case 7:
      efr32_rfp_timer_irq(dev);
      break;
    case 8:
      irq = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);

      if (irq & EFR32_TX_IRQ_FRC_MSK)
        efr32_rfp_tx_irq(ctx, irq);
      else if (irq & EFR32_RX_IRQ_FRC_MSK)
        efr32_rfp_rx_irq(ctx, irq);

      /* Clear irqs */
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, irq);
      break;

    case 9:
      irq = cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_IF_ADDR);
      irq &= cpu_mem_read_32(EFM32_RTCC_ADDR + EFM32_RTCC_IEN_ADDR);
      cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_IFC_ADDR, endian_le32(EFM32_RTCC_IFC_MASK));
      if (irq & EFM32_RTCC_IEN_CC(1))
        {
#ifdef CONFIG_DEVICE_CLOCK_GATING
          /* Wakeup Radio if sleeping */
          for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT -1; i++)
            dev_clock_sink_gate(&ctx->pv.clk_ep[i], DEV_CLOCK_EP_POWER_CLOCK);
          ctx->sleep = 0;
#endif
          x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
          x |= 0x2;
          cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
        }
      else if (irq & EFM32_RTCC_IEN_CC(2))
        {
         cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, 0);
         cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_RXDIS);
         kroutine_init_deferred(&ctx->pv.kr, &efr32_rfp_ldc);
         kroutine_exec(&ctx->pv.kr);
        }
      break;
    default:
      printk("irq: %d\n", idx);
      while(1);
      break;
  }

  lock_release(&dev->lock);
}

/* Transceiver is doing nothing */

static void efr32_rfp_idle(struct radio_efr32_rfp_ctx_s *ctx)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  /* Wakeup Radio if sleeping */
  if (ctx->sleep)
    {
      for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++)
        dev_clock_sink_gate(&pv->clk_ep[i], DEV_CLOCK_EP_POWER_CLOCK);
      ctx->sleep = 0;
    }
#endif

  /* Ensure RAC is OFF */
  if (efr32_check_rac_off(ctx) == 0)
    return;

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&ctx->queue);

  if (!rq)
    rq = ctx->rx_cont;

  if (!rq)
    {
#ifdef CONFIG_DEVICE_CLOCK_GATING
      if (ctx->sleep == 0)
        {
          /* Radio is allowed to go to sleep */
          for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++)
            dev_clock_sink_gate(&pv->clk_ep[i], DEV_CLOCK_EP_POWER);
          ctx->sleep = 1;
        }
#endif
      return;
    } 

  if (efr32_rf_config(ctx, rq) || efr32_pkt_config(ctx, rq))
    {
      /* Unsupported configuration */
      if (rq->type != DEV_RFPACKET_RQ_RX_CONT)
        return efr32_rfp_end_rq(ctx, -ENOTSUP);

      /* Rx continuous */	
      rq->error = -ENOTSUP;
      dev_rfpacket_rq_done(ctx->rx_cont);
      return;
    }

  rq->base.drvdata = NULL;
  
  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_TX_FAIR:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_LBT);
      break;
    case DEV_RFPACKET_RQ_TX:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX);
      break;
    case DEV_RFPACKET_RQ_RX_CONT:
    case DEV_RFPACKET_RQ_RX:
      return efr32_rfp_start_rx(ctx, rq);
    default:
      abort();
  }

  if (rq->tx_size > EFR32_RADIO_THRESHOLD)
    {
      kroutine_init_deferred(&pv->kr, &efr32_rfp_tx);
      kroutine_exec(&pv->kr);
      return;
    }

  return efr32_rfp_start_tx(ctx);
}

static DEV_RFPACKET_REQUEST(efr32_radio_request)
{
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  va_list vl;
  va_start(vl, accessor);

  while(1)
  {
    struct dev_rfpacket_rq_s *rq = va_arg(vl, struct dev_rfpacket_rq_s *);

    if (rq == NULL)
      break;

    assert(rq != ctx->rx_cont);
    assert(rq != ctx->next_rx_cont);

    efr32_radio_printk("R %d %d\n", rq->type, ctx->state);

    rq->error = 0;

    if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
      {
        rq->base.drvdata = NULL;
        switch (ctx->state)
        {
          case EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX:
            if (ctx->next_rx_cont && ctx->next_rx_cont != ctx->rx_cont)
              dev_rfpacket_rq_done(ctx->next_rx_cont);
            ctx->next_rx_cont = rq;
            break;
          case EFR32_RFP_STATE_RXC_PENDING_RX:
            assert(ctx->next_rx_cont == NULL);
            assert(ctx->rx_cont);
            efr32_rfp_disable(ctx);
            efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX);
            ctx->next_rx_cont = rq;
            break;
          case EFR32_RFP_STATE_RXC:
            assert(ctx->next_rx_cont == NULL);
            assert(ctx->rx_cont);
            /* Replace the current RX continous */
            efr32_rfp_disable(ctx);
            dev_rfpacket_rq_done(ctx->rx_cont);
          case EFR32_RFP_STATE_IDLE:
            ctx->rx_cont = rq;
            efr32_rfp_idle(ctx);
            break;
          case EFR32_RFP_STATE_WAIT_FOR_IDLE:
            ctx->rx_cont = rq;
            break;
          default:
            assert(ctx->next_rx_cont == NULL);
            if (ctx->rx_cont)
              dev_rfpacket_rq_done(ctx->rx_cont);
            ctx->rx_cont = rq;
            break;
        }
      }
    else
      {
        bool_t empty = dev_rq_queue_isempty(&ctx->queue);
        dev_rfpacket_rq_pushback(&ctx->queue, rq);
        rq->base.drvdata = ctx;

        if (empty)
          {
            switch (ctx->state)
            {
              case EFR32_RFP_STATE_RXC_PENDING_RX:
                /* A RX packet read is pending */
                assert(rq->deadline == 0);
                assert(ctx->rx_cont);
                efr32_rfp_disable(ctx);
                efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX);
                ctx->next_rx_cont = ctx->rx_cont;
                break;
              case EFR32_RFP_STATE_RXC:
                assert(rq->deadline == 0);
                assert(ctx->next_rx_cont == NULL);
                assert(ctx->rx_cont);
                /* Pause the current RX continous */
                efr32_rfp_disable(ctx);
              case EFR32_RFP_STATE_IDLE:
                efr32_rfp_idle(ctx);
                break;
              default:
                break;
            }
          }
      }
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_RFPACKET_CANCEL(efr32_radio_cancel)
{
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;

  error_t err = -EBUSY;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq == ctx->rx_cont)
    {
      switch (ctx->state)
      {
        case EFR32_RFP_STATE_RXC_PENDING_RX:
          assert(ctx->next_rx_cont == NULL);
          assert(ctx->rx_cont);
          efr32_rfp_disable(ctx);
          efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX);
          break;
        case EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX:
          if (rq == ctx->next_rx_cont)
            ctx->next_rx_cont = NULL;
          break;
        case EFR32_RFP_STATE_RXC:
          assert(ctx->next_rx_cont == NULL);
          assert(ctx->rx_cont);
          efr32_rfp_disable(ctx);
          efr32_radio_set_state(ctx, EFR32_RFP_STATE_IDLE);
          ctx->rx_cont = NULL;
          err = 0;
          break;
        default:
          err = 0;
          ctx->rx_cont = NULL;
          if (rq == ctx->next_rx_cont)
            ctx->next_rx_cont = NULL;
          break;
      }
    }
  else if(rq == ctx->next_rx_cont)
    {
      switch (ctx->state)
      {
        case EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX:
          ctx->next_rx_cont = NULL;
          if (rq != ctx->rx_cont)
            err = 0;
        default:
          break;
      }
    }
  else if (rq->base.drvdata == ctx)
    {
      err = 0;
      rq->base.drvdata = NULL;
      dev_rfpacket_rq_remove(&ctx->queue, rq);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define efr32_radio_use dev_use_generic
#define efr32_radio_stats (dev_rfpacket_stats_t*)&dev_driver_notsup_fcn

/**************************** TIMER PART ********************************/

static DEV_TIMER_CANCEL(efr32_rfpacket_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == ctx)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_timer_rq_prev(&pv->pti.queue, rq) == NULL);

      if (first)
        rqnext = dev_timer_rq_next(&pv->pti.queue, rq);

      dev_timer_rq_remove(&pv->pti.queue, rq);
      rq->base.drvdata = NULL;

      if (first)
        {
          efr32_protimer_disable_compare(&pv->pti, EFR32_PROTIMER_CHANNEL);

          if (rqnext != NULL)
            {
              dev_timer_value_t value = efr32_protimer_get_value(&pv->pti);
              /* start next request, raise irq on race condition */
              efr32_protimer_request_start(&pv->pti, value, rqnext->deadline, EFR32_PROTIMER_CHANNEL);
            }
        }

      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_REQUEST(efr32_rfpacket_timer_request)
{
  struct device_s *dev = accessor->dev;

  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct efr32_protimer_s *pti = &pv->pti;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rev && rq->rev != pti->rev)
    err = -EAGAIN;
  else
    {
      /* Start timer if needed */
      if (dev->start_count == 0)
        efr32_protimer_start_counter(pti);

      dev_timer_value_t value = efr32_protimer_get_value(pti);

      if (rq->delay)
        rq->deadline = value + rq->delay;
      if (rq->deadline <= value)
        err = -ETIMEDOUT;
      else
        {
          dev->start_count |= 1;
          dev_timer_rq_insert(&pti->queue, rq);
          rq->base.drvdata = ctx;

          /* start request, raise irq on race condition */
          if (dev_timer_rq_prev(&pti->queue, rq) == NULL)
            efr32_protimer_request_start(pti, value, rq->deadline, EFR32_PROTIMER_CHANNEL);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(efr32_rfpacket_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  LOCK_SPIN_IRQ(&dev->lock);

  *value = efr32_protimer_get_value(&pv->pti);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_TIMER_CONFIG(efr32_rfpacket_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  uint32_t r = 1;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  cfg->freq = pv->freq;

  if (res)
    {
      if (dev->start_count)
        {
          err = -EBUSY;
          r = res;
        }
      else
        {
          /* Resolution can not be modified */
          if (res != 1)
            err = -ERANGE;
          pv->pti.rev += 2;
        }
    }

  if (cfg)
    {
      cfg->rev = pv->pti.rev;
      cfg->res = r;
      cfg->cap = pv->pti.cap;
      cfg->max = 0xffffffffffffffffULL;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static void efr32_rfp_timer_irq(struct device_s *dev)
{
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct efr32_protimer_s *pti = &pv->pti;

  while (1)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IF_ADDR));
      irq &= endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR));

      if (!irq)
        break;

      cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFC_ADDR, endian_le32(irq));

      uint32_t x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
      x = EFR32_RAC_STATUS_STATE_GET(x);  

      /* TX timeout */
      if (irq & EFR32_PROTIMER_IF_LBTFAILURE)
        {
          assert(ctx->state == EFR32_RFP_STATE_TX_LBT);
          /* Check RAC state */
          if (x == EFR32_RAC_STATUS_STATE_RXFRAME)
          {
            ctx->state = EFR32_RFP_STATE_TX_LBT_RX_ONGOING;
	    /* A packet is being received */
            break;
          }
          efr32_rfp_disable(ctx);
          efr32_rfp_try_restart_tx(ctx);
        }

      /* Compare channel RX end interrupt */ 
      if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL))
       {
         /* RX timeout */
         efr32_protimer_disable_compare(pti, EFR32_PROTIMER_RX_STOP_CHANNEL);
         if (x != EFR32_RAC_STATUS_STATE_OFF)
           efr32_rfp_disable(ctx);

         efr32_rfp_end_rq(ctx, 0);
       }

      /* Timer class interrupts */
      if (!(irq & (EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL) | EFR32_PROTIMER_IF_WRAPCNTOF)))
        break;

#if EFR32_PROTIMER_HW_WIDTH < 64
      /* Compare channel interrupt */ 
      if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL))
         efr32_protimer_disable_compare(pti, EFR32_PROTIMER_CHANNEL);

      /* Update the software part of the counter */
      if (irq & EFR32_PROTIMER_IF_WRAPCNTOF)
        pti->swvalue++;
#endif

      /* CC channel irq */
      while (1)
        {
          struct dev_timer_rq_s *rq = dev_timer_rq_head(&pti->queue);
          if (rq == NULL)
            break;

          dev_timer_value_t value = efr32_protimer_get_value(pti);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!efr32_protimer_request_start(pti, value, rq->deadline, EFR32_PROTIMER_CHANNEL))
              break;

          dev_timer_rq_remove(&pti->queue, rq);
          efr32_protimer_disable_compare(pti, EFR32_PROTIMER_CHANNEL);
          rq->base.drvdata = NULL;

          lock_release(&dev->lock);
          dev_timer_rq_done(rq);
          lock_spin(&dev->lock);
        }
    }
}

#ifdef CONFIG_DRIVER_EFR32_DEBUG
static void efr32_rfp_cfg_rac_dbg(struct radio_efr32_rfp_ctx_s *ctx)
{
  uint32_t x;

  /* Set PC9 to PC11 in output */
  uint32_t a = 0x4000a008 + 2 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x444 << 4));

  /* Configure PRS channel 9/10/11 on PC9/10/11 */
  x = EFR32_PRS_ROUTEPEN_CHPEN(9)  |
      EFR32_PRS_ROUTEPEN_CHPEN(10) |
      EFR32_PRS_ROUTEPEN_CHPEN(11);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTEPEN_ADDR, x);

  x = EFR32_PRS_ROUTELOC2_CH9LOC(14) |
      EFR32_PRS_ROUTELOC2_CH10LOC(4) |
      EFR32_PRS_ROUTELOC2_CH11LOC(4);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTELOC2_ADDR, x);

  /* PC9 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(RAC);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 1); //RAC TX

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(9), x);  

  /* PC10 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(RAC);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 2); //RAC RX

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(10), x);  

  /* PC11 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(MODEML);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 0); //MODEM FRAME DET

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(11), x);  
}

static void efr32_rfp_cfg_protimer_dbg(struct radio_efr32_rfp_ctx_s *ctx)
{
  uint32_t x;

  /* Set PC9 to PC11 in output */
  uint32_t a = 0x4000a008 + 2 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x444 << 4));

  /* Configure PRS channel 9/10/11 on PC9/10/11 */
  x = EFR32_PRS_ROUTEPEN_CHPEN(9)  |
      EFR32_PRS_ROUTEPEN_CHPEN(10) |
      EFR32_PRS_ROUTEPEN_CHPEN(11);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTEPEN_ADDR, x);

  x = EFR32_PRS_ROUTELOC2_CH9LOC(14) |
      EFR32_PRS_ROUTELOC2_CH10LOC(4) |
      EFR32_PRS_ROUTELOC2_CH11LOC(4);

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_ROUTELOC2_ADDR, x);

  /* PC9 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(PROTIMERH);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 7); //PROTIMERCC1

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(9), x);  

  /* PC10 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(PROTIMERH);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 2); //PROTIMERCC2

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(10), x);  

  /* PC11 */
  x = EFR32_PRS_CH_CTRL_SOURCESEL(PROTIMERL);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, 0); //PROTIMERPOF

  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(11), x);  
}
#endif

static DEV_INIT(efr32_radio_init);
static DEV_CLEANUP(efr32_radio_cleanup);

DRIVER_DECLARE(efr32_radio_drv, 0, "EFR32 RFP radio", efr32_radio,
    DRIVER_RFPACKET_METHODS(efr32_radio),
    DRIVER_TIMER_METHODS(efr32_rfpacket_timer));

DRIVER_REGISTER(efr32_radio_drv,
    DEV_ENUM_FDTNAME_ENTRY("efr32:radio"));

static DEV_INIT(efr32_radio_init)
{
  struct radio_efr32_rfp_ctx_s	*ctx;

  /* allocate private driver data */
  ctx = mem_alloc(sizeof(*ctx), (mem_scope_sys));

  if (!ctx)
    return -ENOMEM;

  memset(ctx, 0, sizeof(*ctx));

  struct radio_efr32_ctx_s *pv = &ctx->pv;

  dev->drv_pv = ctx;
  pv->dev = dev;

  if (dev_drv_clock_init(dev, &pv->pti.clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
    goto err_clk;

  for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++)
    {
      if (dev_drv_clock_init(dev, &pv->clk_ep[i], i, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
        goto err_clk;
    }

  pv->freq.num = EFR32_RADIO_HFXO_CLK;
  pv->freq.denom = 1;

  device_get_res_freq(dev, &pv->freq, 0);

  assert(pv->freq.denom == 1);

  /* Queues init */
  dev_rq_queue_init(&ctx->queue);
  dev_rq_pqueue_init(&pv->pti.queue);

  /* Sequencer code initialisaton */
  efr32_radio_seq_init(pv, seqcode, 4 * seqcode_size);

  /* Turn off radio */
  efr32_rfp_disable(ctx);

  /* Timer init */
  efr32_protimer_init(&pv->pti);
  /* Add specific rfpacket protimer initialisation */
  uint32_t x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR);

  x |=  EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL) |
        EFR32_PROTIMER_IF_LBTFAILURE;

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR, x);

  /* Timeout counter 0 synchronized on PRECNTOF
     Timeout counter 0 prescaler decremented on PRECNTOF */
  x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR);
  x |= EFR32_PROTIMER_CTRL_TOUT_SRC(0, PRECNTOF);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR, x);


  // TX/RX buffers initialisation 
  x = bit_ctz32(EFR32_RADIO_RFP_BUFFER_SIZE) - 6;

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(0), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(0), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(1), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(1), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(2), (uint32_t)pv->rx_length_buffer);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(2), 0);

  /* Clear buffer */
  for (uint8_t i = 0; i < 4; i++)
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(i), EFR32_BUFC_CMD_CLEAR);

  /* Enable irq */
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, EFR32_FRC_IF_MASK); 
  x = EFR32_TX_IRQ_FRC_MSK | EFR32_RX_IRQ_FRC_MSK;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR, x);

#ifdef CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV
  /* setup pinmux */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, ">sel? >nsel?", loc, NULL, NULL))
    goto err_mem;

  x = EFR32_MODEM_ROUTEPEN_ANT0PEN | EFR32_MODEM_ROUTEPEN_ANT1PEN;
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_ROUTEPEN_ADDR, x);
  x = EFR32_MODEM_ROUTELOC1_ANT0LOC(loc[0]) | EFR32_MODEM_ROUTELOC1_ANT1LOC(loc[1]);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_ROUTELOC1_ADDR, x);
#endif

  /* Interrupt init */
  device_irq_source_init(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT,
      &efr32_radio_irq);

  if (device_irq_source_link(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT, -1))
    goto err_mem;

  dev->start_count |= 1;
  efr32_protimer_start_counter(&pv->pti);

#ifdef CONFIG_DRIVER_EFR32_DEBUG
//  efr32_rfp_cfg_rac_dbg(ctx);
//  efr32_radio_debug_init(pv);
  efr32_radio_debug_port(pv, 0xF); 
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
  ensure(!device_get_accessor_by_path(&pv->clock.base, NULL, "recmu*", DRIVER_CLASS_CMU));
  /* Radio is allowed to go to sleep */
  for (uint8_t i = 0; i < EFR32_RADIO_CLK_EP_COUNT; i++)
    dev_clock_sink_gate(&pv->clk_ep[i], DEV_CLOCK_EP_POWER);
  ctx->sleep = 1;
#endif
  return 0;

err_clk:
  for (uint8_t i = 0; i < 8; i++)
    dev_drv_clock_cleanup(dev, &pv->clk_ep[i]);
err_mem:
  mem_free(ctx);
  return -1;
}

static DEV_CLEANUP(efr32_radio_cleanup)
{
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  if (!dev_rq_queue_isempty(&ctx->queue) || ctx->rx_cont || 
       dev->start_count > 0)
    return -EBUSY;

  device_irq_source_unlink(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT);

  dev_rq_pqueue_destroy(&pv->pti.queue);
  dev_rq_queue_destroy(&ctx->queue);

#ifdef CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV
  device_iomux_cleanup(dev);
#endif

  mem_free(ctx);

  return 0;
}


static error_t efr32_rfp_fsk_init(struct radio_efr32_rfp_ctx_s *ctx)
{
  uint32_t base[2] = {EFR32_FRC_ADDR, EFR32_RADIO_SEQ_RAM_ADDR};
  uint16_t i = 0;

  const uint32_t generated[] = {
    0x00013008UL, 0x0100AC13UL,
    0x00023030UL, 0x00104000UL,
    /*    3034 */ 0x00000003UL,
    0x00013040UL, 0x00000000UL,
    0x000140A0UL, 0x0F0027AAUL,
    0x000140B8UL, 0x0023C000UL,
    0x000140F4UL, 0x00001020UL,
    0x00024134UL, 0x00000880UL,
    /*    4138 */ 0x000087F6UL,
    0x00024140UL, 0x008800E0UL,
    /*    4144 */ 0x4D52E6C1UL,
    0x00044160UL, 0x00000000UL,
    /*    4164 */ 0x00000000UL,
    /*    4168 */ 0x00000006UL,
    /*    416C */ 0x00000006UL,
    0x00086014UL, 0x00000010UL,
    /*    6018 */ 0x04000000UL,
    /*    601C */ 0x0002C00FUL,
    /*    6020 */ 0x00005000UL,
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV
    /*    6024 */ (EFR32_MODEM_CTRL3_ANTDIVMODE(ANTSELRSSI) |
#else
    /*    6024 */ (EFR32_MODEM_CTRL3_ANTDIVMODE(ANTENNA0) |
#endif
                   EFR32_MODEM_CTRL3_TSAMPMODE(ON) |
                   EFR32_MODEM_CTRL3_TSAMPDEL(3) |
                   EFR32_MODEM_CTRL3_TSAMPLIM(8)),
    /*    6028 */ 0x03000000UL,
    /*    602C */ 0x00000000UL,
    /*    6030 */ 0x00000000UL,
    0x00066050UL, 0x00FF7C83UL,
    /*    6054 */ 0x00000F73UL,
    /*    6058 */ 0x00000160UL,
    /*    605C */ 0x00140011UL,
    /*    6060 */ 0x000075E3UL,
    /*    6064 */ 0x00000000UL,
    0x000C6078UL, 0x11A0071BUL,
    /*    607C */ 0x00000000UL,
    /*    6080 */ 0x003B0373UL,
    /*    6084 */ 0x00000000UL,
    /*    6088 */ 0x00000000UL,
    /*    608C */ 0x22140A04UL,
    /*    6090 */ 0x4F4A4132UL,
    /*    6094 */ 0x00000000UL,
    /*    6098 */ 0x00000000UL,
    /*    609C */ 0x00000000UL,
    /*    60A0 */ 0x00000000UL,
    /*    60A4 */ 0x00000000UL,
    0x000760E4UL, 0x04000080UL,
    /*    60E8 */ 0x00000000UL,
    /*    60EC */ 0x07830464UL,
    /*    60F0 */ 0x3AC81388UL,
    /*    60F4 */ 0x000A209CUL,
    /*    60F8 */ 0x00206100UL,
    /*    60FC */ 0x123556B7UL,
    0x00036104UL, 0x00108000UL,
    /*    6108 */ 0x29043020UL,
    /*    610C */ 0x0040BB88UL,
    0x00016120UL, 0x00000000UL,
    0x00086130UL, 0x00FA53E8UL,
    /*    6134 */ 0x00000000UL,
    /*    6138 */ 0x00000000UL,
    /*    613C */ 0x00000000UL,
    /*    6140 */ 0x00000000UL,
    /*    6144 */ 0x00000000UL,
    /*    6148 */ 0x00000000UL,
    /*    614C */ 0x00000001UL,
    0x00077014UL, 0x000270FEUL,
    /*    7018 */ 0x00000300UL,
    /*    701C */ 0x834A0060UL,
    /*    7020 */ 0x00000000UL,
    /*    7024 */ 0x00000082UL,
    /*    7028 */ 0x00000000UL,
    /*    702C */ 0x000000D5UL,
    0x00027048UL, 0x0000383EUL,
    /*    704C */ 0x000025BCUL,
    0x00037070UL, 0x00120105UL,
    /*    7074 */ 0x00083019UL,
    /*    7078 */ 0x006D8480UL,
    0xFFFFFFFFUL,
  };

  while(1)
    {
      uint32_t v0 = generated[i];

      if (v0 == 0xFFFFFFFF)
        break;

      uint32_t offset = (v0 >> 24) & 0xFF;
      uint32_t count = (v0 >> 16) & 0xFF;
      uint32_t idx = 0;

      assert(offset< 2);

      while(count--)
        {
          uint32_t addr = (v0 & 0xFFFF) + idx;

          addr |= base[offset];

          uint32_t v1 = generated[i + 1];
          cpu_mem_write_32(addr, v1);
//          printk("0x%x 0x%x\n", addr, v1);

          i += 1;
          idx += 4;
        }
      i += 1;
    }

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CTRL_ADDR, 0x100ac3f);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CALCTRL_ADDR, 0x42801);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCDACCTRL_ADDR, 0x23);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, 0x18);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOTUNING_ADDR, 0x7f);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOGAIN_ADDR, 0x23);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_AUXVCDACCTRL_ADDR, 0x7);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CAPCALCYCLECNT_ADDR, 0x3b);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CTRL_ADDR, 0x380);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSCTRL_ADDR, 0xc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSIDLESEQ_ADDR, 0xbc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_HFXORETIMECTRL_ADDR, 0x540);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PRESC_ADDR, 0x7);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHENCTRL_ADDR, 0x87f2);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHREGCTRL_ADDR, 0x3636d80);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL_ADDR, 0x1147b);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL_ADDR, 0x2e);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCAL_ADDR, 0x24);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_AUXCTRL_ADDR, 0x1688030);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFENCTRL_ADDR, 0x1001000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGRFENCTRL0_ADDR, 0xb0000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGLNAMIXCTRL_ADDR, 0x186db00);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPACTRL0_ADDR, 0x491fdfe0);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPAPKDCTRL_ADDR, 0x108d000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL0_ADDR, 0x7000445);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL1_ADDR, 0x84523);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCTRL_ADDR, 0x34);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCAL_ADDR, 0x30191a);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACAL_ADDR, 0x46404301);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RCTUNE_ADDR, 0x1f001f);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_APC_ADDR, 0xff000000L);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_FREQOFFEST_ADDR, 0xed9d0000L);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPCTRL_ADDR, 0x600);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPLEV_ADDR, 0x960000);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCCOMP_ADDR, 0x33);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_BREST_ADDR, 0x2c7);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_POE_ADDR, 0x1ff);

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_SLICECODE_ADDR, 0xca86543);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE1_ADDR, 0x6543210);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE2_ADDR, 0x18b52507);
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE3_ADDR, 0x25183dcd);

  return 0;
}

static error_t efr32_rfp_sigfox_init(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CTRL_ADDR, 0x100ac3f);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CALCTRL_ADDR, 0x42801);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCDACCTRL_ADDR, 0x23);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_IFFREQ_ADDR, 0x10231c);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR, 0x3);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOTUNING_ADDR, 0x7b);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOGAIN_ADDR, 0x22);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_AUXVCDACCTRL_ADDR, 0x7);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CAPCALCYCLECNT_ADDR, 0x7f);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CTRL_ADDR, 0x380);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSCTRL_ADDR, 0xc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSIDLESEQ_ADDR, 0xbc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_HFXORETIMECTRL_ADDR, 0x540);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_WAITMASK_ADDR, 0x1);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_WAITSNSH_ADDR, 0x1);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_STIMER_ADDR, 0xd);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_STIMERCOMP_ADDR, 0x4a45);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_VECTADDR_ADDR, 0x21000000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SEQCTRL_ADDR, 0x1);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PRESC_ADDR, 0x7);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SR_ADDR(0), 0x40010049);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHREGCTRL_ADDR, 0x3636d80);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_VCOCTRL_ADDR, 0xf0027aa);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL_ADDR, 0x1147b);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL_ADDR, 0x2e);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCAL_ADDR, 0x24);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LPFCTRL_ADDR, 0xf3c000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_AUXCTRL_ADDR, 0x1360010);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFENCTRL_ADDR, 0x1ff057);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFENCTRL0_ADDR, 0x3000000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGRFENCTRL0_ADDR, 0x3000000);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGLNAMIXCTRL_ADDR, 0x186db00);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPACTRL0_ADDR, 0x423fffe8);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPAPKDCTRL_ADDR, 0x108d700);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL0_ADDR, 0x7000445);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL1_ADDR, 0x84523);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCTRL_ADDR, 0x35);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCAL_ADDR, 0x301e15);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LNAMIXCTRL1_ADDR, 0x880);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACTRL_ADDR, 0x87f6);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACAL_ADDR, 0x44400246);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFFILTCTRL_ADDR, 0x880020);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFADCCTRL_ADDR, 0x4d52e6c1);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RCTUNE_ADDR, 0x220022);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_APC_ADDR, 0xff000000L);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL1_ADDR, 0x6);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL1_ADDR, 0x6);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_FREQOFFEST_ADDR, 0x2a8c0000);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MIXCTRL_ADDR, 0x10);
  /* DBPSK */
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL0_ADDR, 0x20000c0);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL1_ADDR, 0x12c80c);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL4_ADDR, 0x13000000);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL5_ADDR, 0x600);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TXBR_ADDR, 0x1bb80);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RXBR_ADDR, 0x841);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CF_ADDR, 0x8010eb4);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_PRE_ADDR, 0x92);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SYNC0_ADDR, 0x1ac0);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TIMING_ADDR, 0x22c004b);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MODINDEX_ADDR, 0x2F4);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING0_ADDR, 0x3b261100);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING1_ADDR, 0x7b726350);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING2_ADDR, 0x7f);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPCTRL_ADDR, 0xf00);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPLEV_ADDR, 0x960000);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCCOMP_ADDR, 0x33);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCESTI_ADDR, 0x3f7e00a3);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SRCCHF_ADDR, 0xcc15088bL);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD0_ADDR, 0x7830464);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD1_ADDR, 0x3ac81388);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSACTRL_ADDR, 0x62090);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VITERBIDEMOD_ADDR, 0x206100);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG0_ADDR, 0x208556b7);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DIGMIXCTRL_ADDR, 0x10bb3f);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG1_ADDR, 0x3020);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTTRACK_ADDR, 0xbb88);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_BREST_ADDR, 0x2);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_POE_ADDR, 0x1ff);

  return 0;
}

