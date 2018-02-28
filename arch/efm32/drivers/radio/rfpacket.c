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

/* LBT parameters */
#define EFR32_ETSI_LBT_TIME              5000ULL     /*us*/
#define EFR32_MAX_LBT_RETRY_COUNT        4

/* Protimer Radio class parameters */  
#define EFR32_PROTIMER_RX_START_CHANNEL  1
#define EFR32_PROTIMER_RX_STOP_CHANNEL   2
#define EFR32_PROTIMER_TX_CHANNEL        3

#define EFR32_RADIO_THRESHOLD 16

#define EFR32_RADIO_RFP_BUFFER_SIZE 256

enum efr32_radio_rfp_state
{
  EFR32_RFP_STATE_IDLE,
  EFR32_RFP_STATE_RX,
  EFR32_RFP_STATE_RX_PENDING_RX,
  EFR32_RFP_STATE_RX_DONE_PENDING_RX,
  EFR32_RFP_STATE_RXC,                             /* 4 */
  EFR32_RFP_STATE_RXC_STOPPING,
  EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX,
  EFR32_RFP_STATE_RXC_PENDING_RX,
  EFR32_RFP_STATE_TX,
  EFR32_RFP_STATE_TX_LBT,                          /* 9 */
  EFR32_RFP_STATE_TX_LBT_PENDING_RXC,
  EFR32_RFP_STATE_TX_PENDING_RX,
  EFR32_RFP_STATE_TX_PENDING_RX_AND_RXC,
  EFR32_RFP_STATE_TX_PENDING_RXC,
  EFR32_RFP_STATE_TX_DONE_PENDING_RX,
  EFR32_RFP_STATE_TX_DONE_PENDING_RX_AND_RXC,
  EFR32_RFP_STATE_CANCEL_DONE_PENDING_RX,          /* 16 */
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
  bool_t                        rx_available;
  enum efr32_radio_rfp_state    state;
  /* Current config */
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;
});

STRUCT_COMPOSE(radio_efr32_rfp_ctx_s, pv);

static void efr32_rfp_idle(struct radio_efr32_rfp_ctx_s *ctx);
static void efr32_rfp_timer_irq(struct device_s *dev);
static void efr32_rfp_end_rq(struct radio_efr32_rfp_ctx_s *ctx, error_t err);
static error_t efr32_rfp_init(struct radio_efr32_rfp_ctx_s *ctx);

static void efr32_radio_set_state(struct radio_efr32_rfp_ctx_s *pv, enum efr32_radio_rfp_state state)
{
  efr32_radio_printk("drv: st %d\n", state);
  pv->state = state;
}

static inline error_t efr32_rf_config(struct radio_efr32_rfp_ctx_s *ctx,
                                      struct dev_rfpacket_rq_s *rq)
{
  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

  if ((rfcfg != ctx->rf_cfg) || rfcfg->cache.dirty)
  /* Test if new RF configuration or previous configuration modified */
    {
      ctx->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)rfcfg;

      if (rfcfg->mod != DEV_RFPACKET_GFSK)
	return -ENOTSUP;

      if (rfcfg->frequency != 868000000)
	return -ENOTSUP;

      if (rfcfg->drate != 38400)
	return -ENOTSUP;
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

  ctx->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pkcfg;

  if (rq->pk_cfg->format != DEV_RFPACKET_FMT_SLPC)
    return -ENOTSUP;

  const struct dev_rfpacket_pk_cfg_basic_s *cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

  /** Configure Sync Word */

  uint8_t sw = cfg->sw_len + 1;

  if ((sw >> 5) || (sw % 4))
    return -ENOTSUP;
  
  uint32_t x = EFR32_MODEM_CTRL1_SYNCBITS(cfg->sw_len);
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

  x = EFR32_MODEM_PRE_BASE(cfg->pb_pattern_len);

  uint32_t msk = (1 << (cfg->pb_pattern_len + 1)) - 1;
  uint32_t preamble = cfg->pb_pattern & msk;

  if (preamble == (0xAAAAAAAA & msk))
    EFR32_MODEM_PRE_BASEBITS_SET(x, 1); /* TYPE 1010 */
  else if (preamble == (0x55555555 & msk))
    EFR32_MODEM_PRE_BASEBITS_SET(x, 0); /* TYPE 0101 */
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
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_INIT_ADDR, 0x0);

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
  uint32_t x;

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TOUTCNTTOP_ADDR(0), EFR32_PROTIMER_TOUTCNTTOP_PCNTTOP(4));
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_TOUTCOMP_ADDR(0), EFR32_PROTIMER_TOUTCOMP_PCNTCOMP(1));

  /* Time granularity is based on T0UF */

  x = /* Time between 2 retries */
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

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
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

static void efr32_rfp_start_tx(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFR32_RADIO_SEQ_RAM_ADDR + 0x1F00, 0);

  struct dev_rfpacket_rq_s *rq;
  rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&ctx->queue));

  /* Check RAC state */
  assert(EFR32_RAC_STATUS_STATE_GET(endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR))) == EFR32_RAC_STATUS_STATE_OFF);

  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR); 

  /* Fill buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEDATA_ADDR(0), rq->tx_size);

  uint8_t *p = (uint8_t *)rq->tx_buf;

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

  /* Check RAC state */
  while(1)
  /* Use state change interrupt here or schedule a timer interrupt instead of active waiting */
    {
      x = endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR));
      if (EFR32_RAC_STATUS_STATE_GET(x) == EFR32_RAC_STATUS_STATE_OFF)
        break;
    }
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
    return efr32_rfp_end_rq(ctx, 0);

  efr32_protimer_request_start(&pv->pti, time, ctx->deadline, EFR32_PROTIMER_RX_STOP_CHANNEL);
  efr32_protimer_request_start(&pv->pti, time, time, EFR32_PROTIMER_RX_START_CHANNEL);

  uint32_t x = EFR32_PROTIMER_RXCTRL_RXSETEVENT(0, ALWAYS) |
               EFR32_PROTIMER_RXCTRL_RXCLREVENT(0, ALWAYS) |
               EFR32_PROTIMER_RXCTRL_RXSETEVENT(1, CC1)    |     
               EFR32_PROTIMER_RXCTRL_RXCLREVENT(1, CC2);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_RXCTRL_ADDR, x);

  /* Check for race condition */
  time = efr32_protimer_get_value(&pv->pti);

  if (start < time)
   {
      /* Enable RX */
     x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
     x |= 0x2;
     cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
   }

  time = efr32_protimer_get_value(&pv->pti);

  if (ctx->deadline > time)
    return;

  efr32_rfp_disable(ctx);
  x = EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFS_ADDR, x);
}

static void efr32_rfp_rx_cfg(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFR32_RADIO_SEQ_RAM_ADDR + 0x1F00, 0);
  /* Check RAC state */
  assert(EFR32_RAC_STATUS_STATE_GET(endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR))) == EFR32_RAC_STATUS_STATE_OFF);

  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR); 
}

static void efr32_rfp_start_rx(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq)
{
  uint32_t x;
  efr32_rfp_rx_cfg(ctx);

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_RX_CONT:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC);
      /* Enable RX */
      x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
      x |= 0x2;
      cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
      break;
    case DEV_RFPACKET_RQ_RX:
      efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX);
      /* Scheduled RX */
      efr32_rfp_start_rx_scheduled(ctx, rq);
      break;
    default:
      assert(1);
  }

  return;
}


static inline void efr32_rfp_process_group(struct radio_efr32_rfp_ctx_s *ctx, bool_t group)
{
  struct dev_rfpacket_rq_s * rq;

  while (1)
  {
    rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&ctx->queue));

    if (!rq || rq->err_group != group)
      break;

    rq->err = -ECANCELED;
    dev_request_queue_pop(&ctx->queue);
    kroutine_exec(&rq->base.kr);
  }
}

static void efr32_rfp_try_restart_rx(struct radio_efr32_rfp_ctx_s *ctx)
{
  efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX);
  efr32_rfp_disable(ctx);

  dev_timer_value_t time = efr32_protimer_get_value(&ctx->pv.pti);

  if (ctx->deadline <= time)
    return;

  /* Enable RX */
  uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
  x |= 0x2;
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
  
}

static void efr32_rfp_read_done(struct radio_efr32_rfp_ctx_s *ctx)
{
  switch (ctx->state)
    {
      case EFR32_RFP_STATE_TX_DONE_PENDING_RX_AND_RXC:
        if (ctx->rx_cont)
          kroutine_exec(&ctx->rx_cont->base.kr);
        ctx->rx_cont = ctx->next_rx_cont;
        ctx->next_rx_cont = NULL;
      case EFR32_RFP_STATE_RX_DONE_PENDING_RX:
      case EFR32_RFP_STATE_TX_DONE_PENDING_RX:
      case EFR32_RFP_STATE_CANCEL_DONE_PENDING_RX:
        efr32_rfp_idle(ctx);
        break;
      case EFR32_RFP_STATE_RX_PENDING_RX:
      case EFR32_RFP_STATE_RX:
        efr32_rfp_try_restart_rx(ctx);
        break;
      case EFR32_RFP_STATE_TX_PENDING_RX:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_LBT);
        break;
      case EFR32_RFP_STATE_TX_PENDING_RX_AND_RXC:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_PENDING_RXC);
        break;
      case EFR32_RFP_STATE_RXC_PENDING_RX:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC);
        break;
      case EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC_STOPPING);
        break;
      default:
        abort();
    }
}

static void efr32_rfp_read_packet(struct radio_efr32_rfp_ctx_s *ctx, bool_t locked)
{
  struct dev_rfpacket_rx_s *rx = ctx->rx;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  rx->timestamp = 0;
  rx->snr = 0;
  rx->carrier = 0;
 
  uint8_t *p = (uint8_t *)rx->buf;

  /* Read packet */
  for (uint16_t i = 0; i < rx->size; i++)
    p[i] = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));

  rx->rssi = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));

  kroutine_exec(&rx->kr);

  if (locked)
    efr32_rfp_read_done(ctx);
  else
    {
      LOCK_SPIN_IRQ(&pv->dev->lock);
      efr32_rfp_read_done(ctx);
      LOCK_RELEASE_IRQ(&pv->dev->lock);
    }
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

static bool_t efr32_rfp_discard_rx_test(struct radio_efr32_rfp_ctx_s *ctx)
{
  return 0;
}

static void efr32_rfp_end_rq(struct radio_efr32_rfp_ctx_s *ctx, error_t err)
{
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&ctx->queue));

  assert(rq && rq->type != DEV_RFPACKET_RQ_RX_CONT);

  dev_request_queue_pop(&ctx->queue);
  kroutine_exec(&rq->base.kr);

  rq->err = err;

  if (rq->err)
    efr32_rfp_process_group(ctx, rq->err_group);

  switch (ctx->state)
    {
      case EFR32_RFP_STATE_RX_PENDING_RX:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_RX_DONE_PENDING_RX);
        return;
      case EFR32_RFP_STATE_TX_PENDING_RX_AND_RXC:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_DONE_PENDING_RX_AND_RXC);
        return;
      case EFR32_RFP_STATE_TX_PENDING_RX:
        efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_DONE_PENDING_RX);
        return;
      case EFR32_RFP_STATE_TX_PENDING_RXC:
        if (ctx->rx_cont)
          kroutine_exec(&ctx->rx_cont->base.kr);
        ctx->rx_cont = ctx->next_rx_cont;
        ctx->next_rx_cont = NULL;
      default:
        return efr32_rfp_idle(ctx);
    }
}

static void efr32_rfp_rx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;
  struct dev_rfpacket_rq_s *rq = NULL;

  if (irq & EFR32_FRC_IF_RXDONE)
    {
      uint16_t size = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_READDATA_ADDR(1));

      cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);

      uint32_t status = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_STATUS_ADDR(1));
      status = EFR32_BUFC_STATUS_BYTES_GET(status);

      assert(EFR32_BUFC_STATUS_BYTES_GET(status) >= size);

       switch (ctx->state)
       {
         case EFR32_RFP_STATE_TX_LBT:
         case EFR32_RFP_STATE_TX_PENDING_RXC:
           efr32_radio_printk("rx during tx\n");
           if (efr32_rfp_discard_rx_test(ctx))
             return;
         case EFR32_RFP_STATE_RXC_STOPPING:
         case EFR32_RFP_STATE_RXC:
           rq = ctx->rx_cont;
           break;
         case EFR32_RFP_STATE_RX:
           rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&ctx->queue));
           break;
         default:
           abort();
       }
      
       assert(rq);

       struct dev_rfpacket_rx_s * rx = rq->rx_alloc(rq, size);
       
       if (rx == NULL)
       /* Flush RX fifo */
         {
           cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);
           cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(2), EFR32_BUFC_CMD_CLEAR);
           return;
         }

       assert(rx->size == size);
       ctx->rx = rx;
       
       if (size > EFR32_RADIO_THRESHOLD)
         {
           switch (ctx->state)
           {
             case EFR32_RFP_STATE_TX_LBT:
               efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_PENDING_RX);
               break;
             case EFR32_RFP_STATE_TX_PENDING_RXC:
               efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_PENDING_RX_AND_RXC);
               break;
             case EFR32_RFP_STATE_RXC_STOPPING:
               efr32_radio_set_state(ctx, EFR32_RFP_STATE_RXC_STOPPING_PENDING_RX);
               break;
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
       else
         return efr32_rfp_read_packet(ctx, 1);
    }
  else
    efr32_rfp_try_restart_rx(ctx);
}

static inline void efr32_rfp_tx_irq(struct radio_efr32_rfp_ctx_s *ctx, uint32_t irq)
{
//  efr32_rfp_print_debug("After tx", ctx);
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&ctx->queue));
  assert(rq);

  switch (ctx->state)
  {
    case EFR32_RFP_STATE_TX_PENDING_RX_AND_RXC:
    case EFR32_RFP_STATE_TX_PENDING_RXC:
    case EFR32_RFP_STATE_TX_PENDING_RX:
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
      rq->err = 0;
      rq->tx_timestamp = 0;
    }
  else
    /* Clear buffer */
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR);

  return efr32_rfp_end_rq(ctx, 0);
}

static DEV_IRQ_SRC_PROCESS(efr32_radio_irq)
{
  struct device_s *dev = ep->base.dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;

  lock_spin(&dev->lock);

  uint32_t irq = 0;

  switch (ep - ctx->pv.irq_ep)
  {
    case 0:
      irq = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IEN_ADDR);
      cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_IFC_ADDR, irq);
      efr32_radio_printk("modem irq: 0x%x\n", irq);
      break;
    case 1:
    case 2:
      efr32_radio_printk("Rac irq: 0x%x\n", cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_IF_ADDR));
      break;
    case 3:
      irq = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IEN_ADDR);
      efr32_radio_printk("bufc irq: 0x%x\n", irq);
      break;
    case 7:
      efr32_rfp_timer_irq(dev);
      break;
    case 8:
      irq = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);

      assert(irq);

      efr32_radio_printk("frc irq: 0x%x\n", irq);

      if (irq & EFR32_TX_IRQ_FRC_MSK)
        efr32_rfp_tx_irq(ctx, irq);
      else if (irq & EFR32_RX_IRQ_FRC_MSK)
        efr32_rfp_rx_irq(ctx, irq);

      /* Clear irqs */
      cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, irq);
      break;

    default:
      efr32_radio_printk("irq: %d\n", ep - ctx->pv.irq_ep);
      abort();
      break;
  }

  lock_release(&dev->lock);
}


/* Transceiver is doing nothing */

static void efr32_rfp_idle(struct radio_efr32_rfp_ctx_s *ctx)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  efr32_radio_set_state(ctx, EFR32_RFP_STATE_IDLE);
  efr32_rfp_disable(ctx);

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&ctx->queue));

  if (!rq)
    rq = ctx->rx_cont;

  if (!rq)
    return;

  efr32_radio_printk("drv: Start rq: %d\n", rq->type);

  if (efr32_pkt_config(ctx, rq) || efr32_rf_config(ctx, rq))
    {
      /* Unsupported configuration */
      if (rq->type != DEV_RFPACKET_RQ_RX_CONT)
        return efr32_rfp_end_rq(ctx, -ENOTSUP);

      /* Rx continuous */	
      rq->err = -ENOTSUP;
      kroutine_exec(&ctx->rx_cont->base.kr);
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

static void efr32_rfp_cancel(struct radio_efr32_rfp_ctx_s *ctx)
{
  efr32_rfp_disable(ctx);

  if (ctx->rx_cont)
    kroutine_exec(&ctx->rx_cont->base.kr);

  ctx->rx_cont = NULL;
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

    efr32_radio_printk("drv: New Request %d %d\n", rq->type, ctx->state);

    rq->err = 0;

    if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
      {
        rq->base.drvdata = NULL;
        switch (ctx->state)
        {
          case EFR32_RFP_STATE_RXC_PENDING_RX:
          case EFR32_RFP_STATE_RXC:
            assert(ctx->rx_cont);
            efr32_rfp_cancel(ctx);
            ctx->rx_cont = rq;
            break;
          case EFR32_RFP_STATE_TX_PENDING_RXC:
            if (ctx->next_rx_cont)
              kroutine_exec(&ctx->next_rx_cont->base.kr);
            ctx->next_rx_cont = rq;
            break;
          case EFR32_RFP_STATE_TX_LBT:
            efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_PENDING_RXC);
            ctx->next_rx_cont = rq;
            break;
          case EFR32_RFP_STATE_IDLE:
            assert(ctx->rx_cont == NULL);
            ctx->rx_cont = rq;
            efr32_rfp_idle(ctx);
            break;
          default:
            assert(ctx->next_rx_cont == NULL);
            if (ctx->rx_cont)
              kroutine_exec(&ctx->rx_cont->base.kr);
            ctx->rx_cont = rq;
            break;
        }
      }
    else
      {
        bool_t empty = dev_request_queue_isempty(&ctx->queue);
        dev_request_queue_pushback(&ctx->queue, dev_rfpacket_rq_s_base(rq));
        rq->base.drvdata = ctx;

        if (empty)
          {
            switch (ctx->state)
            {
              case EFR32_RFP_STATE_RXC:
                assert(ctx->next_rx_cont == NULL);
              case EFR32_RFP_STATE_RXC_PENDING_RX:
                assert(ctx->rx_cont);
                assert(rq->deadline == 0);
                /* The next rx continous is the same as the current one */
                ctx->next_rx_cont = ctx->rx_cont;
                efr32_rfp_cancel(ctx);
                break;
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

  efr32_radio_printk("drv: Cancel %d\n", ctx->state);

  error_t err = -EBUSY;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq == ctx->rx_cont)
    {
      switch (ctx->state)
      {
        case EFR32_RFP_STATE_RXC_PENDING_RX:
        case EFR32_RFP_STATE_RXC:
          assert(ctx->next_rx_cont == NULL);
          assert(ctx->rx_cont);
          efr32_rfp_disable(ctx);
          ctx->rx_cont = NULL;
          err = 0;
          break;
        case EFR32_RFP_STATE_TX_PENDING_RXC:
          if (ctx->next_rx_cont)
            kroutine_exec(&ctx->next_rx_cont->base.kr);
          ctx->next_rx_cont = rq;
          break;
        case EFR32_RFP_STATE_TX_LBT:
          efr32_radio_set_state(ctx, EFR32_RFP_STATE_TX_PENDING_RXC);
          if (rq == ctx->next_rx_cont)
            ctx->next_rx_cont = NULL;
          break;
        case EFR32_RFP_STATE_TX_LBT_PENDING_RXC:
          assert(rq != ctx->next_rx_cont);
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
        case EFR32_RFP_STATE_TX_LBT_PENDING_RXC:
          ctx->next_rx_cont = NULL;
          if (rq != ctx->rx_cont)
            err = 0;
          break;

        default:
          abort();
      }
    }
  else if (rq->base.drvdata == ctx)
    {
      err = 0;
      rq->base.drvdata = NULL;
      dev_request_queue_remove(&ctx->queue, dev_rfpacket_rq_s_base(rq));
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define efr32_radio_use dev_use_generic
#define efr32_radio_stats (dev_rfpacket_stats_t*)&dev_driver_notsup_fcn

/**************************** TIMER PART ********************************/

static inline void efr32_timer_raise_irq(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFS_ADDR,
                   endian_le32(EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL)));
}

static inline void efr32_timer_stop_counter(struct radio_efr32_rfp_ctx_s *ctx)
{
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, endian_le32(EFR32_PROTIMER_CMD_STOP));
#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IF_ADDR))
      & (EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL) | EFR32_PROTIMER_IF_WRAPCNTOF))
    return;
  dev_clock_sink_gate(&ctx->pv.clk_ep, DEV_CLOCK_EP_POWER);
#endif
}

static DEV_TIMER_CANCEL(efr32_rfpacket_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  error_t err = -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata == ctx)
    {
      struct dev_timer_rq_s *rqnext = NULL;
      bool_t first = (dev_request_pqueue_prev(&pv->pti.queue, dev_timer_rq_s_base(rq)) == NULL);

      if (first)
        rqnext = dev_timer_rq_s_cast(dev_request_pqueue_next(&pv->pti.queue, dev_timer_rq_s_base(rq)));

      dev_timer_pqueue_remove(&pv->pti.queue, dev_timer_rq_s_base(rq));
      rq->rq.drvdata = NULL;

      if (first)
        {
          efr32_protimer_disable_compare(&pv->pti, EFR32_PROTIMER_CHANNEL);

          if (rqnext != NULL)
            {
              dev_timer_value_t value = efr32_protimer_get_value(&pv->pti);
              /* start next request, raise irq on race condition */
              if (efr32_protimer_request_start(&pv->pti, rqnext->deadline, value, EFR32_PROTIMER_CHANNEL))
                efr32_timer_raise_irq(ctx);
            }
          else
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                efr32_timer_stop_counter(ctx);
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
          dev_timer_pqueue_insert(&pti->queue, dev_timer_rq_s_base(rq));
          rq->rq.drvdata = ctx;

          /* start request, raise irq on race condition */
          if (dev_request_pqueue_prev(&pti->queue, dev_timer_rq_s_base(rq)) == NULL)
            if (efr32_protimer_request_start(pti, rq->deadline, value, EFR32_PROTIMER_CHANNEL))
              efr32_timer_raise_irq(ctx);
        }

      if (dev->start_count == 0)
        efr32_timer_stop_counter(ctx);
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

      /* TX timeout */
      if (irq & EFR32_PROTIMER_IF_LBTFAILURE)
        efr32_rfp_end_rq(ctx, -ETIMEDOUT);

      /* Compare channel RX end interrupt */ 
      if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL))
       {
         /* RX timeout */
         efr32_protimer_disable_compare(pti, EFR32_PROTIMER_RX_STOP_CHANNEL);
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
          struct dev_timer_rq_s *rq;
          rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pti->queue));
          if (rq == NULL)
            {
              dev->start_count &= ~1;
              if (dev->start_count == 0)
                {
                  efr32_timer_stop_counter(ctx);
                  return;
                }
              break;
            }

          dev_timer_value_t value = efr32_protimer_get_value(pti);

          /* setup compare for first request */
          if (rq->deadline > value)
            if (!efr32_protimer_request_start(pti, rq->deadline, value, EFR32_PROTIMER_CHANNEL))
              break;

          dev_timer_pqueue_remove(&pti->queue, dev_timer_rq_s_base(rq));
          efr32_protimer_disable_compare(pti, EFR32_PROTIMER_CHANNEL);
          rq->rq.drvdata = NULL;

          lock_release(&dev->lock);
          kroutine_exec(&rq->rq.kr);
          lock_spin(&dev->lock);
        }
    }
}

#ifdef CONFIG_DRIVER_EFR32_DEBUG
static void efr32_rfp_cfg_prs_dbg(struct radio_efr32_rfp_ctx_s *ctx)
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


  if (efr32_rfp_init(ctx))
    goto err_mem;

  uint32_t x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR);

  x |=  EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_RX_STOP_CHANNEL) |
        EFR32_PROTIMER_IF_LBTFAILURE;

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR, x);

  /* Timeout counter 0 synchronized on PRECNTOF
     Timeout counter 0 prescaler decremented on PRECNTOF */
  x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR);
  x |= EFR32_PROTIMER_CTRL_TOUT_SRC(0, PRECNTOF);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR, x);

  pv->freq.num = EFR32_RADIO_HFXO_CLK;
  pv->freq.denom = 1;

  device_get_res_freq(dev, &pv->freq, 0);

  assert(pv->freq.denom == 1);

  /* Queues init */
  dev_request_queue_init(&ctx->queue);
  dev_request_pqueue_init(&pv->pti.queue);

  /* Interrupt init */
  device_irq_source_init(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT,
      &efr32_radio_irq);

  if (device_irq_source_link(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT, -1))
    goto err_mem;

  efr32_protimer_start_counter(&pv->pti);

#ifdef CONFIG_DRIVER_EFR32_DEBUG
  efr32_rfp_cfg_prs_dbg(ctx);
  efr32_radio_debug_init(pv);
#endif

  return 0;

err_mem:
  mem_free(ctx);
  return -1;
}

static DEV_CLEANUP(efr32_radio_cleanup)
{
  struct radio_efr32_rfp_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  if (!dev_request_queue_isempty(&ctx->queue) || ctx->rx_cont || 
       dev->start_count > 0)
    return -EBUSY;

  device_irq_source_unlink(dev, pv->irq_ep, EFR32_RADIO_IRQ_COUNT);

  dev_request_pqueue_destroy(&pv->pti.queue);
  dev_request_queue_destroy(&ctx->queue);

  mem_free(ctx);

  return 0;
}

static error_t efr32_rfp_init(struct radio_efr32_rfp_ctx_s *ctx)
{
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

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CTRL_ADDR, EFR32_SYNTH_CTRL_DITHERDSMINPUT |
                                                           EFR32_SYNTH_CTRL_DITHERDSMOUTPUT(7) |
                                                           EFR32_SYNTH_CTRL_DITHERDAC(3) |
                                                           EFR32_SYNTH_CTRL_LOCKTHRESHOLD(3) |
                                                           EFR32_SYNTH_CTRL_AUXLOCKTHRESHOLD(5) |
                                                           EFR32_SYNTH_CTRL_PRSMUX0(DISABLED) |
                                                           EFR32_SYNTH_CTRL_PRSMUX1(DISABLED) |
                                                           EFR32_SYNTH_CTRL_DEMMODE);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CALCTRL_ADDR, EFR32_SYNTH_CALCTRL_NUMCYCLES(1) |
                                                           EFR32_SYNTH_CALCTRL_CAPCALCYCLEWAIT(CYCLES1) |
                                                           EFR32_SYNTH_CALCTRL_STARTUPTIMING(10) |
                                                           EFR32_SYNTH_CALCTRL_AUXCALCYCLES(4) |
                                                           EFR32_SYNTH_CALCTRL_AUXCALCYCLEWAIT(CYCLES64));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCDACCTRL_ADDR, EFR32_SYNTH_VCDACCTRL_VCDACVAL(35));

  /* Frequency 868 MHz */
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_FREQ_ADDR, EFR32_SYNTH_FREQ_FREQ(35553280));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_IFFREQ_ADDR, EFR32_SYNTH_IFFREQ_IFFREQ(16384) |
                                                               EFR32_SYNTH_IFFREQ_LOSIDE);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR, EFR32_SYNTH_DIVCTRL_LODIVFREQCTRL(LODIV3) |
                                                           EFR32_SYNTH_DIVCTRL_AUXLODIVFREQCTRL(LODIV1));

  /* Channel spacing */
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHSP_ADDR, EFR32_SYNTH_CHSP_CHSP(4096));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOTUNING_ADDR, EFR32_SYNTH_VCOTUNING_VCOTUNING(125));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOGAIN_ADDR, EFR32_SYNTH_VCOGAIN_VCOGAIN(35));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CAPCALCYCLECNT_ADDR, EFR32_SYNTH_CAPCALCYCLECNT_CAPCALCYCLECNT(127));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CTRL_ADDR, EFR32_RAC_CTRL_ACTIVEPOL |
                                                         EFR32_RAC_CTRL_PAENPOL |
                                                         EFR32_RAC_CTRL_LNAENPOL);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSCTRL_ADDR, EFR32_RAC_LVDSCTRL_LVDSCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSIDLESEQ_ADDR, EFR32_RAC_LVDSIDLESEQ_LVDSIDLESEQ(188));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_HFXORETIMECTRL_ADDR, EFR32_RAC_HFXORETIMECTRL_LIMITH(4) |
                                                           EFR32_RAC_HFXORETIMECTRL_LIMITL(5));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_WAITSNSH_ADDR, EFR32_RAC_WAITSNSH_WAITSNSH(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PRESC_ADDR, EFR32_RAC_PRESC_STIMER(7));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHENCTRL_ADDR, EFR32_RAC_SYNTHENCTRL_VCOEN |
                                                                EFR32_RAC_SYNTHENCTRL_LODIVEN |
                                                                EFR32_RAC_SYNTHENCTRL_CHPEN |
                                                                EFR32_RAC_SYNTHENCTRL_LPFEN |
                                                                EFR32_RAC_SYNTHENCTRL_SYNTHCLKEN |
                                                                EFR32_RAC_SYNTHENCTRL_SYNTHSTARTREQ |
                                                                EFR32_RAC_SYNTHENCTRL_CHPLDOEN |
                                                                EFR32_RAC_SYNTHENCTRL_LODIVSYNCCLKEN |
                                                                EFR32_RAC_SYNTHENCTRL_MMDLDOEN |
                                                                EFR32_RAC_SYNTHENCTRL_VCOLDOEN |
                                                                EFR32_RAC_SYNTHENCTRL_VCODIVEN);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHREGCTRL_ADDR, EFR32_RAC_SYNTHREGCTRL_MMDLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_MMDLDOVREFTRIM(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_VCOLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_VCOLDOVREFTRIM(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_CHPLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_CHPLDOVREFTRIM(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_VCOCTRL_ADDR, EFR32_RAC_VCOCTRL_VCOAMPLITUDE(10) |
                                                            EFR32_RAC_VCOCTRL_VCODETAMPLITUDE(10) |
                                                            EFR32_RAC_VCOCTRL_VCODETEN |
                                                            EFR32_RAC_VCOCTRL_VCODETMODE |
                                                            EFR32_RAC_VCOCTRL_VCOAREGCURR(1) |
                                                            EFR32_RAC_VCOCTRL_VCOCREGCURR(2) |
                                                            EFR32_RAC_VCOCTRL_VCODIVCURR(15));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL_ADDR, EFR32_RAC_MMDCTRL_MMDDIVDCDC(123) |
                                                            EFR32_RAC_MMDCTRL_MMDDIVRSDCDC(1) |
                                                            EFR32_RAC_MMDCTRL_MMDDIVRSDIG(1) |
                                                            EFR32_RAC_MMDCTRL_MMDENDCDC |
                                                            EFR32_RAC_MMDCTRL_MMDENRSDCDC |
                                                            EFR32_RAC_MMDCTRL_MMDENRSDIG);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL_ADDR, EFR32_RAC_CHPCTRL_CHPBIAS(6) |
                                                           EFR32_RAC_CHPCTRL_CHPCURR(5));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCAL_ADDR, EFR32_RAC_CHPCAL_PSRC(4) |
                                                           EFR32_RAC_CHPCAL_NSRC(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LPFCTRL_ADDR, EFR32_RAC_LPFCTRL_LPFINPUTCAP(3) |
                                                           EFR32_RAC_LPFCTRL_LPFSWITCHINGEN |
                                                           EFR32_RAC_LPFCTRL_LPFGNDSWITCHINGEN |
                                                           EFR32_RAC_LPFCTRL_LPFBWTX(2));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_AUXCTRL_ADDR, EFR32_RAC_AUXCTRL_CHPCURR(3) |
                                                           EFR32_RAC_AUXCTRL_RXAMP(16) |
                                                           EFR32_RAC_AUXCTRL_LDOAMPCURR(4) |
                                                           EFR32_RAC_AUXCTRL_LDOVREFTRIM(6) |
                                                           EFR32_RAC_AUXCTRL_LPFRES(1));

  /* Sub-G PA configuration */

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGRFENCTRL0_ADDR, EFR32_RAC_SGRFENCTRL0_LNAMIXBIASEN |
                                                           EFR32_RAC_SGRFENCTRL0_LNAMIXLOBIASEN |
                                                           EFR32_RAC_SGRFENCTRL0_LNAMIXRFBIASEN);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGLNAMIXCTRL_ADDR, EFR32_RAC_SGLNAMIXCTRL_CASCODEBIAS(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_LOBIAS(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_VREG(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_RFBIAS(3) |
                                                           EFR32_RAC_SGLNAMIXCTRL_SGREGAMPCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPACTRL0_ADDR, EFR32_RAC_SGPACTRL0_CASCODE(EN3SLICES) |
                                                           EFR32_RAC_SGPACTRL0_SLICE(EN3SLICES) |
                                                           EFR32_RAC_SGPACTRL0_STRIPE(8) |
                                                           EFR32_RAC_SGPACTRL0_DACGLITCHCTRL);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPAPKDCTRL_ADDR, EFR32_RAC_SGPAPKDCTRL_VTHSEL(23) |
                                                           EFR32_RAC_SGPAPKDCTRL_CAPSEL(3) |
                                                           EFR32_RAC_SGPAPKDCTRL_I2VCM(2) |
                                                           EFR32_RAC_SGPAPKDCTRL_PKDBIASTH(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL0_ADDR, EFR32_RAC_SGPABIASCTRL0_LDOBIAS |
                                                           EFR32_RAC_SGPABIASCTRL0_PABIAS(1) |
                                                           EFR32_RAC_SGPABIASCTRL0_BUF0BIAS(1) |
                                                           EFR32_RAC_SGPABIASCTRL0_BUF12BIAS(1) |
                                                           EFR32_RAC_SGPABIASCTRL0_SGDACFILTBANDWIDTH(7));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SGPABIASCTRL1_ADDR, EFR32_RAC_SGPABIASCTRL1_VLDO(3) |
                                                           EFR32_RAC_SGPABIASCTRL1_VLDOFB(2) |
                                                           EFR32_RAC_SGPABIASCTRL1_VCASCODEHV(5) |
                                                           EFR32_RAC_SGPABIASCTRL1_VCASCODELV(4) |
                                                           EFR32_RAC_SGPABIASCTRL1_SGVBATDETTHRESHOLD(2));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCTRL_ADDR, EFR32_RAC_RFBIASCTRL_LDOVREF(4) |
                                                           EFR32_RAC_RFBIASCTRL_LDOAMPCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCAL_ADDR, EFR32_RAC_RFBIASCAL_VREF(22) |
                                                           EFR32_RAC_RFBIASCAL_BIAS(28) |
                                                           EFR32_RAC_RFBIASCAL_TEMPCO(48));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LNAMIXCTRL1_ADDR, EFR32_RAC_LNAMIXCTRL1_TRIMAUXPLLCLK(E0) |
                                                           EFR32_RAC_LNAMIXCTRL1_TRIMVCASLDO |
                                                           EFR32_RAC_LNAMIXCTRL1_TRIMVREGMIN(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACTRL_ADDR, EFR32_RAC_IFPGACTRL_VLDO(3) |
                                                           EFR32_RAC_IFPGACTRL_BANDSEL(SG) |
                                                           EFR32_RAC_IFPGACTRL_CASCBIAS(7) |
                                                           EFR32_RAC_IFPGACTRL_TRIMVCASLDO |
                                                           EFR32_RAC_IFPGACTRL_TRIMVCM(3) |
                                                           EFR32_RAC_IFPGACTRL_TRIMVREGMIN(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACAL_ADDR, EFR32_RAC_IFPGACAL_IRAMP(2) |
                                                           EFR32_RAC_IFPGACAL_IRPHASE(10) |
                                                           EFR32_RAC_IFPGACAL_OFFSETI(64));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFFILTCTRL_ADDR, EFR32_RAC_IFFILTCTRL_CENTFREQ(7) |
                                                           EFR32_RAC_IFFILTCTRL_VCM(2) |
                                                           EFR32_RAC_IFFILTCTRL_VREG(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFADCCTRL_ADDR, EFR32_RAC_IFADCCTRL_REALMODE |
                                                           EFR32_RAC_IFADCCTRL_VLDOSERIES(3) |
                                                           EFR32_RAC_IFADCCTRL_VLDOSERIESCURR(3) |
                                                           EFR32_RAC_IFADCCTRL_VLDOSHUNT(2) |
                                                           EFR32_RAC_IFADCCTRL_VLDOCLKGEN(3) |
                                                           EFR32_RAC_IFADCCTRL_VCM(E2) |
                                                           EFR32_RAC_IFADCCTRL_OTA1CURRENT(2) |
                                                           EFR32_RAC_IFADCCTRL_OTA2CURRENT(2) |
                                                           EFR32_RAC_IFADCCTRL_OTA3CURRENT(2) |
                                                           EFR32_RAC_IFADCCTRL_REGENCLKDELAY(3) |
                                                           EFR32_RAC_IFADCCTRL_ENABLECLK |
                                                           EFR32_RAC_IFADCCTRL_INVERTCLK);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RCTUNE_ADDR, EFR32_RAC_RCTUNE_IFADCRCTUNE(34) |
                                                           EFR32_RAC_RCTUNE_IFFILT(34));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_APC_ADDR, EFR32_RAC_APC_AMPCONTROLLIMITSW(255));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL1_ADDR, EFR32_RAC_CHPCTRL1_BYPREPLDORX |
                                                           EFR32_RAC_CHPCTRL1_TRIMREPLDO(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL1_ADDR, EFR32_RAC_MMDCTRL1_BYPREPLDORX |
                                                           EFR32_RAC_MMDCTRL1_TRIMREPLDO(1));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_FREQOFFEST_ADDR, EFR32_MODEM_FREQOFFEST_FREQOFFEST(9) |
                                                           EFR32_MODEM_FREQOFFEST_CORRVAL(115) |
                                                           EFR32_MODEM_FREQOFFEST_SOFTVAL(221));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MIXCTRL_ADDR, EFR32_MODEM_MIXCTRL_MODE(NORMAL) |
                                                           EFR32_MODEM_MIXCTRL_DIGIQSWAPEN);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL0_ADDR, EFR32_MODEM_CTRL0_MAPFSK(MAP0) |
                                                           EFR32_MODEM_CTRL0_CODING(NRZ) |
                                                           EFR32_MODEM_CTRL0_MODFORMAT(FSK2) |
                                                           EFR32_MODEM_CTRL0_DSSSSHIFTS(NOSHIFT) |
                                                           EFR32_MODEM_CTRL0_DSSSDOUBLE(DIS) |
                                                           EFR32_MODEM_CTRL0_DIFFENCMODE(DIS) |
                                                           EFR32_MODEM_CTRL0_SHAPING(EVENLENGTH) |
                                                           EFR32_MODEM_CTRL0_DEMODRAWDATASEL(DIS) |
                                                           EFR32_MODEM_CTRL0_FRAMEDETDEL(DEL0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL2_ADDR, EFR32_MODEM_CTRL2_SQITHRESH(200) |
                                                           EFR32_MODEM_CTRL2_TXPINMODE(OFF) |
                                                           EFR32_MODEM_CTRL2_DATAFILTER(LEN7) |
                                                           EFR32_MODEM_CTRL2_RATESELMODE(NOCHANGE) |
                                                           EFR32_MODEM_CTRL2_DMASEL(SOFT));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL3_ADDR, EFR32_MODEM_CTRL3_PRSDINSEL(PRSCH0) |
                                                           EFR32_MODEM_CTRL3_ANTDIVMODE(ANTENNA0) |
                                                           EFR32_MODEM_CTRL3_TSAMPMODE(ON) |
                                                           EFR32_MODEM_CTRL3_TSAMPDEL(3) |
                                                           EFR32_MODEM_CTRL3_TSAMPLIM(8));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL4_ADDR, EFR32_MODEM_CTRL4_ADCSATLEVEL(CONS64));

  /* Baudrate 38400 bit/s */
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TXBR_ADDR, EFR32_MODEM_TXBR_TXBRNUM(31875) |
                                                             EFR32_MODEM_TXBR_TXBRDEN(255));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RXBR_ADDR, EFR32_MODEM_RXBR_RXBRNUM(19) |
                                                           EFR32_MODEM_RXBR_RXBRDEN(27) |
                                                           EFR32_MODEM_RXBR_RXBRINT(3));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CF_ADDR, EFR32_MODEM_CF_DEC0(DF3) |
                                                           EFR32_MODEM_CF_DEC1(44) |
                                                           EFR32_MODEM_CF_CFOSR(CF7) |
                                                           EFR32_MODEM_CF_DEC1GAIN(ADD0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TIMING_ADDR, EFR32_MODEM_TIMING_TIMTHRESH(27) |
                                                           EFR32_MODEM_TIMING_TIMINGBASES(7) |
                                                           EFR32_MODEM_TIMING_OFFSUBNUM(13) |
                                                           EFR32_MODEM_TIMING_OFFSUBDEN(8) |
                                                           EFR32_MODEM_TIMING_FASTRESYNC(DIS));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MODINDEX_ADDR, EFR32_MODEM_MODINDEX_MODINDEXM(20) |
                                                           EFR32_MODEM_MODINDEX_MODINDEXE(27) |
                                                           EFR32_MODEM_MODINDEX_FREQGAINE(3) |
                                                           EFR32_MODEM_MODINDEX_FREQGAINM(7));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING0_ADDR, EFR32_MODEM_SHAPING0_COEFF0(4) |
                                                           EFR32_MODEM_SHAPING0_COEFF1(10) |
                                                           EFR32_MODEM_SHAPING0_COEFF2(20) |
                                                           EFR32_MODEM_SHAPING0_COEFF3(34));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING1_ADDR, EFR32_MODEM_SHAPING1_COEFF4(50) |
                                                           EFR32_MODEM_SHAPING1_COEFF5(65) |
                                                           EFR32_MODEM_SHAPING1_COEFF6(74) |
                                                           EFR32_MODEM_SHAPING1_COEFF7(79));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPCTRL_ADDR, EFR32_MODEM_RAMPCTRL_RAMPRATE2(6));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPLEV_ADDR, EFR32_MODEM_RAMPLEV_RAMPLEV2(150));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCCOMP_ADDR, EFR32_MODEM_DCCOMP_DCESTIEN |
                                                           EFR32_MODEM_DCCOMP_DCCOMPEN |
                                                           EFR32_MODEM_DCCOMP_DCCOMPGEAR(3) |
                                                           EFR32_MODEM_DCCOMP_DCLIMIT(FULLSCALE));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCESTI_ADDR, EFR32_MODEM_DCESTI_DCCOMPESTIVALI(51) |
                                                               EFR32_MODEM_DCESTI_DCCOMPESTIVALQ(32679));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SRCCHF_ADDR, EFR32_MODEM_SRCCHF_SRCRATIO1(128) |
                                                               EFR32_MODEM_SRCCHF_SRCRATIO2(1024));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD0_ADDR, EFR32_MODEM_DSATHD0_SPIKETHD(100) |
                                                           EFR32_MODEM_DSATHD0_UNMODTHD(4) |
                                                           EFR32_MODEM_DSATHD0_FDEVMINTHD(12) |
                                                           EFR32_MODEM_DSATHD0_FDEVMAXTHD(120));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD1_ADDR, EFR32_MODEM_DSATHD1_POWABSTHD(5000) |
                                                           EFR32_MODEM_DSATHD1_POWRELTHD(DISABLED) |
                                                           EFR32_MODEM_DSATHD1_DSARSTCNT(2) |
                                                           EFR32_MODEM_DSATHD1_RSSIJMPTHD(6) |
                                                           EFR32_MODEM_DSATHD1_FREQLATDLY(1) |
                                                           EFR32_MODEM_DSATHD1_PWRFLTBYP |
                                                           EFR32_MODEM_DSATHD1_AMPFLTBYP |
                                                           EFR32_MODEM_DSATHD1_PWRDETDIS);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSACTRL_ADDR, EFR32_MODEM_DSACTRL_DSAMODE(DISABLED) |
                                                           EFR32_MODEM_DSACTRL_ARRTHD(7) |
                                                           EFR32_MODEM_DSACTRL_ARRTOLERTHD0(2) |
                                                           EFR32_MODEM_DSACTRL_ARRTOLERTHD1(4) |
                                                           EFR32_MODEM_DSACTRL_FREQAVGSYM |
                                                           EFR32_MODEM_DSACTRL_DSARSTON);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VITERBIDEMOD_ADDR, EFR32_MODEM_VITERBIDEMOD_VITERBIKSI1(64) |
                                                           EFR32_MODEM_VITERBIDEMOD_VITERBIKSI2(48) |
                                                           EFR32_MODEM_VITERBIDEMOD_VITERBIKSI3(32));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG0_ADDR, EFR32_MODEM_VTCORRCFG0_EXPECTPATT(349879) |
                                                           EFR32_MODEM_VTCORRCFG0_EXPSYNCLEN(8) |
                                                           EFR32_MODEM_VTCORRCFG0_BUFFHEAD(2));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DIGMIXCTRL_ADDR, EFR32_MODEM_DIGMIXCTRL_DIGMIXFREQ(32768) |
                                                           EFR32_MODEM_DIGMIXCTRL_DIGMIXMODE);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG1_ADDR, EFR32_MODEM_VTCORRCFG1_CORRSHFTLEN(32) |
                                                           EFR32_MODEM_VTCORRCFG1_VTFRQLIM(192));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTTRACK_ADDR, EFR32_MODEM_VTTRACK_FREQTRACKMODE(DISABLED) |
                                                           EFR32_MODEM_VTTRACK_TIMTRACKTHD(2) |
                                                           EFR32_MODEM_VTTRACK_TIMEACQUTHD(238) |
                                                           EFR32_MODEM_VTTRACK_TIMEOUTMODE |
                                                           EFR32_MODEM_VTTRACK_TIMGEAR(GEAR0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_BREST_ADDR, EFR32_MODEM_BREST_BRESTINT(7) |
                                                           EFR32_MODEM_BREST_BRESTNUM(11));
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_POE_ADDR, EFR32_MODEM_POE_POEI(511));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_RSSI_ADDR, EFR32_AGC_RSSI_RSSIFRAC(3) |
                                                           EFR32_AGC_RSSI_RSSIINT(149));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_FRAMERSSI_ADDR, EFR32_AGC_FRAMERSSI_FRAMERSSIINT(128));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, EFR32_AGC_CTRL1_RSSIPERIOD(3));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL2_ADDR, EFR32_AGC_CTRL2_HYST(3) |
                                                           EFR32_AGC_CTRL2_FASTLOOPDEL(10) |
                                                           EFR32_AGC_CTRL2_CFLOOPDEL(26) |
                                                           EFR32_AGC_CTRL2_ADCRSTFASTLOOP(GAINREDUCTION) |
                                                           EFR32_AGC_CTRL2_ADCRSTSTARTUP);

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_IFPEAKDET_ADDR, EFR32_AGC_IFPEAKDET_PKDTHRESH1(2) |
                                                           EFR32_AGC_IFPEAKDET_PKDTHRESH2(8));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_RFPEAKDET_ADDR, EFR32_AGC_RFPEAKDET_RFPKDTHRESH1(5) |
                                                           EFR32_AGC_RFPEAKDET_RFPKDTHRESH2(13));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAINRANGE_ADDR, EFR32_AGC_GAINRANGE_MAXGAIN(62) |
                                                           EFR32_AGC_GAINRANGE_MINGAIN(112));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAININDEX_ADDR, EFR32_AGC_GAININDEX_NUMINDEXPGA(12) |
                                                           EFR32_AGC_GAININDEX_NUMINDEXDEGEN(3) |
                                                           EFR32_AGC_GAININDEX_NUMINDEXSLICES(6) |
                                                           EFR32_AGC_GAININDEX_NUMINDEXATTEN(18));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_SLICECODE_ADDR, EFR32_AGC_SLICECODE_SLICECODEINDEX0(3) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX1(4) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX2(5) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX3(6) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX4(8) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX5(10) |
                                                           EFR32_AGC_SLICECODE_SLICECODEINDEX6(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE1_ADDR, EFR32_AGC_ATTENCODE1_ATTENCODEINDEX1(1) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX2(2) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX3(3) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX4(4) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX5(5) |
                                                           EFR32_AGC_ATTENCODE1_ATTENCODEINDEX6(6));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE2_ADDR, EFR32_AGC_ATTENCODE2_ATTENCODEINDEX7(7) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX8(8) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX9(9) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX10(10) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX11(11) |
                                                           EFR32_AGC_ATTENCODE2_ATTENCODEINDEX12(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE3_ADDR, EFR32_AGC_ATTENCODE3_ATTENCODEINDEX13(13) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX14(14) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX15(15) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX16(16) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX17(17) |
                                                           EFR32_AGC_ATTENCODE3_ATTENCODEINDEX18(18));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAINSTEPLIM_ADDR, EFR32_AGC_GAINSTEPLIM_FASTSTEPDOWN(5) |
                                                           EFR32_AGC_GAINSTEPLIM_FASTSTEPUP(2) |
                                                           EFR32_AGC_GAINSTEPLIM_CFLOOPSTEPMAX(8) |
                                                           EFR32_AGC_GAINSTEPLIM_SLOWDECAYCNT(1) |
                                                           EFR32_AGC_GAINSTEPLIM_ADCATTENMODE(DISABLE));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_LOOPDEL_ADDR, EFR32_AGC_LOOPDEL_PKDWAIT(25) |
                                                           EFR32_AGC_LOOPDEL_IFPGADEL(12) |
                                                           EFR32_AGC_LOOPDEL_LNASLICESDEL(16));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_MININDEX_ADDR, EFR32_AGC_MININDEX_INDEXMINSLICES(18) |
                                                           EFR32_AGC_MININDEX_INDEXMINDEGEN(24) |
                                                           EFR32_AGC_MININDEX_INDEXMINPGA(27));

  struct radio_efr32_ctx_s *pv = &ctx->pv;

  /* Sequencer code initialisaton */
  efr32_radio_seq_init(pv, seqcode, 4 * seqcode_size);

  /* Turn off radio */
  efr32_rfp_disable(ctx);

  /* Timer init */
  efr32_protimer_init(&pv->pti);

  uint32_t x = bit_ctz32(EFR32_RADIO_RFP_BUFFER_SIZE) - 6;

  // TX/RX buffers initialisation 

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(0), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(0), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(1), x);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(1), (uint32_t)ctx->sg_buffer);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(2), (uint32_t)ctx->pv.rx_length_buffer);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(2), 0);

  /* Configure variable length mode */
  x = EFR32_FRC_CTRL_RXFCDMODE(FCDMODE2) |
      EFR32_FRC_CTRL_TXFCDMODE(FCDMODE2) |
      EFR32_FRC_CTRL_BITORDER(MSB) |
      EFR32_FRC_CTRL_BITSPERWORD(7);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);

  x = EFR32_FRC_DFLCTRL_DFLMODE(SINGLEBYTE) |
      EFR32_FRC_DFLCTRL_MINLENGTH(1) |
      EFR32_FRC_DFLCTRL_DFLBITS(8);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, x);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MAXLENGTH_ADDR, EFR32_RADIO_RFP_BUFFER_SIZE - 1);

  /* Clear buffer */
  for (uint8_t i = 0; i < 4; i++)
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(i), EFR32_BUFC_CMD_CLEAR);

  /* Enable irq */
  x = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);
  x |= EFR32_TX_IRQ_FRC_MSK | EFR32_RX_IRQ_FRC_MSK;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, x); 
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR, x);

  return 0;
}

