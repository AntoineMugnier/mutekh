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

    Copyright (c) 2016 Sebastien Cerdan <sebcerdan@gmail.com>


    This driver has some restriction:
      - does not support LORA modulation
      - does not support packet size greater than 64 bytes.
      - does not support packet reception during LBT.
      - does not use PA_BOOST pin for TX.
      - no cache management.

*/

#define LOGK_MODULE_ID "sx12"

#include "sx127x_spi.h"
#include "sx127x_spi.o.h"

#include <mutek/printk.h>
#include <hexo/endian.h>

#define CONFIG_DRIVER_RFPACKET_CLK_SOURCE

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_DEBUG)
# include <mutek/printk.h>
# define dprintk(...) printk("[sx127x] " __VA_ARGS__)
#else
# define dprintk(...) do {} while(0)
#endif

/* ******************************** RF Packet ********************************/

static void sx127x_rfp_end_rq(struct sx127x_private_s *pv, error_t err);
static void sx127x_cancel_rxc(struct sx127x_private_s *pv);
static void sx127x_rfp_idle(struct sx127x_private_s *pv);

static void sx127x_rfp_set_state(struct sx127x_private_s *pv, enum sx127x_state_e state)
{
  dprintk("st %d\n", state);
  pv->state = state;
}

static void sx127x_bytecode_start(struct sx127x_private_s *pv, const void *e, uint16_t mask, ...)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  /* Clear status */
  pv->bc_status &= STATUS_IRQ_MSK;
  bc_set_reg(&srq->vm, STATUS, pv->bc_status);

  assert(pv->bcrun == 0);
  pv->bcrun = 1;

  dprintk("bcstart %d\n", pv->state);

  va_list ap;
  va_start(ap, mask);
  ensure(dev_spi_bytecode_start_va(&pv->spi, srq, e, mask, ap) == 0);
  va_end(ap);
}

static inline void sx127x_config_freq_update(struct sx127x_private_s *pv, uint32_t channel)
{
  uint32_t const freq = (pv->cfg.coeff.freq + channel * pv->cfg.coeff.chan) >> 15;
  uint8_t *f = pv->cfg.freq;

  f[0] = freq >> 16;
  f[1] = freq >> 8;
  f[2] = freq;

  pv->cfg.channel = channel;
}

static inline void sx127x_config_coeff_freq_update(struct sx127x_private_s *pv, const struct dev_rfpacket_rf_cfg_std_s *rf_cfg, uint32_t channel)
{
  pv->cfg.coeff.freq = ((uint64_t)rf_cfg->frequency << (19 + 15)) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;
  pv->cfg.coeff.chan = ((uint64_t)rf_cfg->chan_spacing << (19 + 15)) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;

  sx127x_config_freq_update(pv, channel);
}

struct sx127x_config_bw_s
{
  uint32_t BITFIELD(bw, 24);
  uint8_t  bits;
};

static const struct sx127x_config_bw_s sx127x_config_bw_tbl[] =
{
 /* RX bandwidth given in datasheet is for single side band. */
    { 2 * 2600  , 0x17 },
    { 2 * 3100  , 0x0F },
    { 2 * 3900  , 0x07 },
    { 2 * 5200  , 0x16 },
    { 2 * 6300  , 0x0E },
    { 2 * 7800  , 0x06 },
    { 2 * 10400 , 0x15 },
    { 2 * 12500 , 0x0D },
    { 2 * 15600 , 0x05 },
    { 2 * 20800 , 0x14 },
    { 2 * 25000 , 0x0C },
    { 2 * 31300 , 0x04 },
    { 2 * 41700 , 0x13 },
    { 2 * 50000 , 0x0B },
    { 2 * 62500 , 0x03 },
    { 2 * 83333 , 0x12 },
    { 2 * 100000, 0x0A },
    { 2 * 125000, 0x02 },
    { 2 * 166700, 0x11 },
    { 2 * 200000, 0x09 },
    { 2 * 250000, 0x01 },
};

static uint8_t sx127x_get_bw(uint32_t bw)
{
  for (uint16_t i = 0; i < ARRAY_SIZE(sx127x_config_bw_tbl); i++)
    {
      if (bw <= sx127x_config_bw_tbl[i].bw)
        return sx127x_config_bw_tbl[i].bits;
    }

  return 0;
}

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
static error_t sx127x_build_raw_config(struct sx127x_private_s * __restrict__ pv,
                                       struct dev_rfpacket_rq_s * __restrict__  rq,
                                       uint8_t **p)
{
  const struct dev_rfpacket_pk_cfg_raw_s *pk_cfg = const_dev_rfpacket_pk_cfg_raw_s_cast(rq->pk_cfg);

  /* Set bitbang symbol duration */
  pv->brq.unit.num = pk_cfg->unit.num;
  pv->brq.unit.denom = pk_cfg->unit.denom;

  pv->brq.read_timeout = pk_cfg->timeout;
  pv->brq.sym_width = pk_cfg->sym_width;

  uint8_t * pk = *p;

  pv->cfg.rxcfg |= SX1276_RXCONFIG_RESTARTRXONCOLLISION_ON
                |  SX1276_RXCONFIG_RXTRIGER_RSSI;

  /* Packet config mode */
  *pk++ = 1;
  *pk++ = SX1276_REG_PACKETCONFIG2 | 0x80;
  *pk++ = SX1276_PACKETCONFIG2_DATAMODE_CONTINUOUS;


  /* DIO Mapping */
  *pk++ = 2;
  *pk++ = SX1276_REG_DIOMAPPING1 | 0x80;
  *pk++ = SX1276_DIOMAPPING1_DIO0_11 |               /* Preamble detect */
          SX1276_DIOMAPPING1_DIO1_01;
  *pk++ = SX1276_DIOMAPPING2_MAP_PREAMBLEDETECT;

  *p = pk;

  return 0;
}
#endif

static error_t sx127x_build_pkt_config(struct sx127x_private_s * __restrict__ pv,
                                       struct dev_rfpacket_rq_s * __restrict__  rq,
                                       uint8_t **p)
{
  uint8_t * pk = *p;
  const struct dev_rfpacket_pk_cfg_basic_s *pk_cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

  /* Fifo threshold */

  *pk++ = 1;
  *pk++ = SX1276_REG_FIFOTHRESH | 0x80;
  *pk++ = 0x80 | 0x3F;

  pv->cfg.rxcfg |= SX1276_RXCONFIG_RXTRIGER_PREAMBLEDETECT;

  /* Preamble and Sync Word config */

  if (pk_cfg->pb_pattern_len > 1)
    return -ENOTSUP;

  uint8_t sw = pk_cfg->sw_len + 1;

  /* Sync word size must be either 8 or 16 bits */
  if ((sw & 0x7) || (sw >> 5))
    return -ENOTSUP;

  sw = sw >> 3;

  *pk++ = 3 + sw;
  *pk++ = SX1276_REG_PREAMBLEMSB | 0x80;
  *pk++ = pk_cfg->tx_pb_len >> 8;
  *pk++ = pk_cfg->tx_pb_len;

  uint8_t sc = (pk_cfg->pb_pattern & 2) ? SX1276_SYNCCONFIG_PREAMBLEPOLARITY_AA : SX1276_SYNCCONFIG_PREAMBLEPOLARITY_55;

  /* Store in private data for IO homecontrol mode */
  pv->cfg.sync = sc | SX1276_SYNCCONFIG_SYNC_ON | (sw - 1);

  *pk++ = pv->cfg.sync;

  for (int8_t i=0; i <sw; i++)
    {
      uint8_t x = pk_cfg->sw_value >> ((sw - i - 1) * 8);

      if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
        {
          x = ((x & 0x55) << 1) | ((0xAA & x) >> 1);
          x = ((x & 0x33) << 2) | ((0xCC & x) >> 2);
          x = ((x & 0x0F) << 4) | ((0xF0 & x) >> 4);
        }

      *pk++ = x;
    }

  uint8_t rx_pb_len = pk_cfg->rx_pb_len >> 3;

  /* Saturate RX preamble threshold */
  if (rx_pb_len == 0)
    rx_pb_len = 1;
  else if (rx_pb_len > 3)
    rx_pb_len = 3;

  /*  Sync word + preambule + crc + fifo filling time + 2 bytes margin */
  pv->tbrs = pv->timebit * ((68 + sw) * 8 + IO_MAX_PREAMBLE_SIZE);

  /* Time to achieve preambule detection */
  dev_timer_delay_t t = SX127X_TS_RE_US + SX127X_TS_FS_US + 100;
  dev_timer_init_sec(pv->timer, &pv->tpbrx, 0, t, 1000000);
  pv->tpbrx += pv->timebit * rx_pb_len * 8;

  *pk++ = 1;
  *pk++ = SX1276_REG_PREAMBLEDETECT | 0x80;
  *pk++ = SX1276_PREAMBLEDETECT_DETECTOR_ON | ((rx_pb_len - 1) << 5) | 0xA;

  /* Packet config mode */

  *pk++ = 3;
  *pk++ = SX1276_REG_PACKETCONFIG1 | 0x80;

  if (pk_cfg->crc_seed != 0xffff &&
      !(pk_cfg->crc_seed == 0 && !pk_cfg->crc))
    return -ENOTSUP;

  uint8_t cfg = 0;

  switch (pk_cfg->crc)
    {
    case 0:
      /* NO CRC */;
      break;
    case 0x8005:
      /* CRC16-IBM */;
      cfg = SX1276_PACKETCONFIG1_CRC_ON | SX1276_PACKETCONFIG1_CRCWHITENINGTYPE_IBM;
      break;
    case 0x1021:
      /* CRC16-CCIT */;
      cfg = SX1276_PACKETCONFIG1_CRC_ON | SX1276_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT;
      break;
    default:
      return -ENOTSUP;
    }

  cfg |= SX1276_PACKETCONFIG1_PACKETFORMAT_VARIABLE;

  switch (pk_cfg->encoding)
    {
    case DEV_RFPACKET_CLEAR:
      break;
    case DEV_RFPACKET_MANCHESTER:
      cfg |= SX1276_PACKETCONFIG1_DCFREE_MANCHESTER;
      break;
    default:
      return -ENOTSUP;
    }

  *pk++ = cfg;

  cfg = SX1276_PACKETCONFIG2_DATAMODE_PACKET | CONFIG_DRIVER_RFPACKET_SX127X_MAX_PKT_SIZE >> 8;
  
  /* Enable uart mode */

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
    cfg |= SX1276_PACKETCONFIG2_IOHOME_ON;

  *pk++ = cfg;
  *pk++ = 0xFF;
  
  /* DIO Mapping */
  *pk++ = 2;
  *pk++ = SX1276_REG_DIOMAPPING1 | 0x80;
  *pk++ = SX1276_DIOMAPPING1_DIO2_11;    /* Sync Address */ 
  *pk++ = SX1276_DIOMAPPING2_DIO4_11 |   /* Preamble detect */
          SX1276_DIOMAPPING2_DIO5_10 |   /* Data */
          SX1276_DIOMAPPING2_MAP_PREAMBLEDETECT;

  *p = pk;

  return 0;
}

static error_t sx1276_build_bw_config(struct sx127x_private_s * __restrict__ pv,
                                      const struct dev_rfpacket_rf_cfg_std_s * __restrict__ rf_cfg,
                                      uint8_t **f, uint32_t default_)
{
  uint8_t * rf = *f;

  /* Bandwidth */
  *rf++ = 2;
  *rf++ = SX1276_REG_RXBW | 0x80;

  uint32_t bw = rf_cfg->rx_bw;

  if (bw == 0)
    bw = default_;

  uint8_t bwcode = sx127x_get_bw(bw);
  logk_trace("bandwidth: %u : 0x%02x", bw, bwcode);

  if (!bwcode)
    return -EINVAL;

  *rf++ = bwcode;

  uint32_t rx_tx_freq_err = rf_cfg->freq_err   /* remote freq error */
    + (uint64_t)rf_cfg->frequency * pv->osc_ppb / 1000000000; /* local freq error */

  /* AFC Bandwidth */
  bw += rx_tx_freq_err * 2;
  bwcode = sx127x_get_bw(bw);
  logk_trace("AFC bandwidth: %u : 0x%02x", bw, bwcode);

  *rf++ = bwcode ? bwcode : /* 500k */ 0x01;

  pv->cfg.rxcfg |= SX1276_RXCONFIG_AFCAUTO_ON;

  *rf++ = 1;
  *rf++ = SX1276_REG_AFCFEI;
  *rf++ = SX1276_AFCFEI_AFCAUTOCLEAR_ON;

  *f = rf;

  return 0;
}

static error_t sx127x_build_fsk_config(struct sx127x_private_s * __restrict__ pv,
                                       struct dev_rfpacket_rq_s * __restrict__ rq, uint8_t **f)
{
  const struct dev_rfpacket_rf_cfg_fsk_s * fsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rq->rf_cfg);

  if (sx1276_build_bw_config(pv, &fsk->common, f,   /* carson rule */
                             fsk->common.drate + 2 * fsk->deviation))
    return -EINVAL;

  uint8_t * rf = *f;

  if (fsk->fairtx.mode == DEV_RFPACKET_LBT)
  /* Rssi threshold configuration */
    {
      if (fsk->fairtx.lbt.rssi > 0)
        return -EINVAL;

      *rf++ = 1;
      *rf++ = SX1276_REG_RSSITHRESH | 0x80;
      *rf++ = (abs(fsk->fairtx.lbt.rssi >> 3)) << 1;
      /* Rssi threshold in 0,25dBm steps */
      pv->cfg.rssi_th = fsk->fairtx.lbt.rssi >> 1;
    }

  *rf++ = 2;
  *rf++ = SX1276_REG_FDEVMSB | 0x80;

  /* Deviation */
  uint16_t dev = ((uint64_t)fsk->deviation << 19)/CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;

  if (dev >> 13)
    return -EINVAL;

  endian_be16_na_store(rf, dev);

  rf += 2;

  /* PA config and PA Ramp */
  *rf++ = 2;
  *rf++ = SX1276_REG_PACONFIG | 0x80;
  *rf++ = 0x7f;

  /* Set Gaussian filter to 1 when GFSK */
  if (rq->rf_cfg->mod == DEV_RFPACKET_GFSK) 
    *rf++ = 0x0A | SX1276_PARAMP_MODULATIONSHAPING_01;
  else 
    *rf++ = 0x0A | SX1276_PARAMP_MODULATIONSHAPING_00;

  *f = rf;

  pv->cfg.mode = SX1276_OPMODE_MODULATIONTYPE_FSK |
                 SX1276_OPMODE_LONGRANGEMODE_OFF;

  return 0;
}

static error_t sx127x_build_ask_config(struct sx127x_private_s * __restrict__ pv,
                                       struct dev_rfpacket_rq_s * __restrict__ rq,
                                       uint8_t **f)
{
  const struct dev_rfpacket_rf_cfg_ask_s * ask = const_dev_rfpacket_rf_cfg_ask_s_cast(rq->rf_cfg);
  
  if (ask->symbols > 2)
    return -EINVAL;

  if (sx1276_build_bw_config(pv, &ask->common, f,
                             2 * ask->common.drate))
    return -EINVAL;

  uint8_t * rf = *f;

  if (ask->fairtx.mode == DEV_RFPACKET_LBT)
  /* Rssi threshold configuration */
    {
      if (ask->fairtx.lbt.rssi > 0)
        return -EINVAL;

      *rf++ = 1;
      *rf++ = SX1276_REG_RSSITHRESH | 0x80;
      *rf++ = (abs(ask->fairtx.lbt.rssi >> 3)) << 1;
      /* Rssi threshold in 0,25dBm steps */
      pv->cfg.rssi_th = ask->fairtx.lbt.rssi >> 1;
    }

  *rf++ = 2;
  *rf++ = SX1276_REG_OOKPEAK | 0x80;
  *rf++ = SX1276_OOKPEAK_OOKTHRESHTYPE_PEAK | SX1276_OOKPEAK_BITSYNC_ON;
  /* Sensitivity of the OOK receiver */
  *rf++ = 0x80; 

  /* PA config and PA Ramp */
  *rf++ = 2;
  *rf++ = SX1276_REG_PACONFIG | 0x80;
  *rf++ = 0x7f;
  *rf++ = 0x00;

  *f = rf;

  pv->cfg.mode = SX1276_OPMODE_MODULATIONTYPE_OOK |
                 SX1276_OPMODE_LONGRANGEMODE_OFF;

  return 0;
}

static inline error_t sx127x_check_config(struct sx127x_private_s * __restrict__ pv,
                                          struct dev_rfpacket_rq_s * __restrict__ rq)
{
  assert(rq && (pv->bcrun == 0));

  const struct dev_rfpacket_rf_cfg_s *rfcfg = rq->rf_cfg;

  /* Check that fair tx mode is set when request of type DEV_RFPACKET_RQ_TX_FAIR is used */
  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR)
    {
      switch (rfcfg->mod)
        {
          case DEV_RFPACKET_GFSK:
          case DEV_RFPACKET_FSK:
            {
              const struct dev_rfpacket_rf_cfg_fsk_s * c =
                const_dev_rfpacket_rf_cfg_fsk_s_cast(rfcfg);
              if (c->fairtx.mode == DEV_RFPACKET_NO_FAIRTX)
                return -ENOTSUP;
              break;
            }
          default:
            break;
        }
    }

  bool_t change = 0;
  uint8_t *cfg = pv->cfg.cfg;

  pv->cfg.rxcfg = SX1276_RXCONFIG_AGCAUTO_ON;

  if ((rfcfg != pv->rf_cfg) || rfcfg->cache.dirty)
  /* Test if new RF configuration or previous configuration modified */
    {
      pv->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)rfcfg;
      const struct dev_rfpacket_rf_cfg_fsk_s *cfsk = NULL;
      const struct dev_rfpacket_rf_cfg_ask_s *cask = NULL;
      const struct dev_rfpacket_rf_cfg_std_s *common = NULL;

      switch (rfcfg->mod)
        {
          case DEV_RFPACKET_FSK:
          case DEV_RFPACKET_GFSK:
            if (sx127x_build_fsk_config(pv, rq, &cfg)) {
              return -ENOTSUP;
            }
            cfsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rfcfg);
            common = &cfsk->common;
            break;

          case DEV_RFPACKET_ASK:
            if (sx127x_build_ask_config(pv, rq, &cfg)) {
              return -ENOTSUP;
            }
            cask = const_dev_rfpacket_rf_cfg_ask_s_cast(rfcfg);
            common = &cask->common;
            break;

          default:
            return -ENOTSUP;
        }
      if (common == NULL) {
        return -ENOTSUP;
      }

      /* Compute time to send a bit in us */
      dev_timer_delay_t t = 1000000/common->drate;
      dev_timer_init_sec(pv->timer, &pv->timebit, 0, t, 1000000);

      if (rfcfg->cache.dirty)
        ((struct dev_rfpacket_rf_cfg_s *)rfcfg)->cache.dirty = 0;

      /* Frequency */
      sx127x_config_coeff_freq_update(pv, common, rq->channel);
      
      /* Datarate */
      *cfg++ = 2;
      *cfg++ = SX1276_REG_BITRATEMSB | 0x80;

      uint16_t dr = (CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO/common->drate);
      endian_be16_na_store(cfg, dr);
      cfg += 2;


      change = 1;
    }

  const struct dev_rfpacket_pk_cfg_s *pkcfg = rq->pk_cfg;

  if ((pkcfg != pv->pk_cfg) || pkcfg->cache.dirty)
  /* Test if new Packet configuration or previous configuration modified */
    {
      pv->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pkcfg;

      if (pkcfg->cache.dirty)
        ((struct dev_rfpacket_pk_cfg_s *)pkcfg)->cache.dirty = 0;

      switch (rq->pk_cfg->format)
        {
          case DEV_RFPACKET_FMT_IO:
          case DEV_RFPACKET_FMT_SLPC:
            if (sx127x_build_pkt_config(pv, rq, &cfg))
              return -ENOTSUP;
            break;
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
          case DEV_RFPACKET_FMT_RAW:
            if (rfcfg->mod != DEV_RFPACKET_ASK)
              return -ENOTSUP;
            if (sx127x_build_raw_config(pv, rq, &cfg))
              return -ENOTSUP;
            break;
#endif
          default:
            return -ENOTSUP;
        }
     
      change = 1;
    }

  if (!change)
    return 0;

  /* LNA gain and RXConfig */
  *cfg++ = 2;
  *cfg++ = SX1276_REG_LNA | 0x80;
  *cfg++ = SX1276_LNA_BOOST_ON | SX1276_LNA_GAIN_G1;
  *cfg++ = pv->cfg.rxcfg;

  /* End of data */
  *cfg = 0;

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_DEBUG)
  /* Dump init */
  uint8_t *c = pv->cfg.cfg;
  uint16_t i = 0;

  while(1)
    {
      size_t size = c[i];
      if (!size)
        break;

      printk("0x%x 0x%x ", size, c[i + 1] ^ 0x80);

      for (uint8_t j = 0; j < size; j++)
        printk("0x%x ", c[i + 2 + j]);

      printk("\n");
      i += size + 2;
    }
#endif

  sx127x_bytecode_start(pv, &sx127x_entry_config, SX127X_ENTRY_CONFIG_BCARGS(pv->cfg.cfg));
  return -EAGAIN;
}

static inline void sx127x_rfp_process_group(struct sx127x_private_s *pv, bool_t group)
{
  struct dev_rfpacket_rq_s * rq;

  while (1)
  {
    rq = dev_rfpacket_rq_head(&pv->queue);

    if (!rq || rq->err_group != group)
      break;

    assert(rq->type != DEV_RFPACKET_RQ_RX_CONT);

    rq->error = -ECANCELED;
    rq->base.drvdata = NULL;

    dev_rfpacket_rq_pop(&pv->queue);
    dev_rfpacket_rq_done(rq);
  }
}
/* This compute the next frequency that will be used for channel scanning */
BC_CCALL_FUNCTION(sx127x_next_hopping_freq)
{
  struct sx127x_private_s *pv = (struct sx127x_private_s *)bc_get_reg(ctx, 0);

  assert((pv->state == SX127X_STATE_RX_SCANNING) ||
         (pv->state == SX127X_STATE_RXC_STOP));

  LOCK_SPIN_IRQ(&pv->dev->lock);

  assert(pv->rx_cont);

  uint32_t mask = pv->rx_cont->rx_chan_mask;

  uint32_t tmp = ~((1 << (pv->cfg.channel + 1)) - 1) & mask;
  uint8_t next = tmp ? bit_ctz(tmp) : bit_ctz(mask);

  sx127x_config_freq_update(pv, next);
  
  bc_set_reg((struct bc_context_s *)ctx, 0, next);

  LOCK_RELEASE_IRQ(&pv->dev->lock);

  return 0;
}

BC_CCALL_FUNCTION(sx127x_alloc)
{
  struct sx127x_private_s *pv = (struct sx127x_private_s *)bc_get_reg(ctx, 0);
  struct dev_rfpacket_rq_s *rq;
  uint8_t *p = NULL;

  LOCK_SPIN_IRQ(&pv->dev->lock);

  switch (pv->state)
  {
    case SX127X_STATE_RX:
      rq = dev_rfpacket_rq_head(&pv->queue);
      break;
    case SX127X_STATE_RXC:
    case SX127X_STATE_RX_SCANNING:
      rq = pv->rx_cont;
      break;
    default:
      goto error;
  }

  if (rq == NULL)
    goto error;

  uint8_t size = pv->size;

  switch (rq->pk_cfg->format)
  {
    case DEV_RFPACKET_FMT_IO:
      size = (pv->size & ((1 << 5) - 1)) + 1;
    default:
      if (size == 0 || size > CONFIG_DRIVER_RFPACKET_SX127X_MAX_PKT_SIZE)
        goto error;
      break;
  }

  pv->rxrq = rq->rx_alloc(rq, size);

  struct dev_rfpacket_rx_s *rx = pv->rxrq;

  if ((rx == NULL) || (rx->size != size))
    goto error;

  p = (uint8_t *)rx->buf;

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
    {
      *p++ = pv->size;
       pv->size = size;
    }

error:
  LOCK_RELEASE_IRQ(&pv->dev->lock);
  bc_set_reg((struct bc_context_s *)ctx, 0, (uintptr_t)p);
  return 0;
}

static uint32_t sx127x_set_cmd(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  uint32_t cmd = pv->cfg.sync;

  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
    cmd |= CMD_IO_MODE;

  switch (rq->type)
    {
      case DEV_RFPACKET_RQ_TX_FAIR:
      case DEV_RFPACKET_RQ_TX:{
        int16_t pwr = (rq->tx_pwr >> 3);
        /* Saturate TX power */
        if (pwr > 15)
          pwr = 15;
        if (pwr < 0)
          pwr = 0;
        /* Pa Select is false: Pout = OutputPower */
        cmd |= (0x70 | pwr) << 8;
        break;}
      case DEV_RFPACKET_RQ_RX_CONT:
        if (rq->pk_cfg->format == DEV_RFPACKET_FMT_RAW)
          cmd |= CMD_RXC_RAW;
        if (rq->rx_chan_mask != 1)
        /* Rx scanning on several channels */
          cmd |= CMD_RX_SCANNING;
      case DEV_RFPACKET_RQ_RX:
        if (rq->pk_cfg->format == DEV_RFPACKET_FMT_IO)
          cmd += 1;
        break;
      default:
        abort();
    }

  if (rq->channel == pv->cfg.channel)
    return cmd;

  cmd |= CMD_FREQ_CHANGE;
  sx127x_config_freq_update(pv, rq->channel);
  
  return cmd;
}

static void sx127x_rfp_end_rxrq(struct sx127x_private_s *pv, error_t err, size_t size)
{
  struct dev_rfpacket_rx_s *rx = pv->rxrq;

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_STATS
  pv->stats.rx_count++;
  if (err)
    pv->stats.rx_err_count++;
#endif

  if (rx == NULL)
    return;

  struct dev_rfpacket_rq_s *rq = NULL;

  switch (pv->state)
    {
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
      case SX127X_STATE_RX_RAW:
      case SX127X_STATE_RX_RAW_PENDING_STOP:
        rq = dev_rfpacket_rq_head(&pv->queue);
        break;
      case SX127X_STATE_RXC_RAW:
      case SX127X_STATE_RXC_RAW_PENDING_STOP:
        rq = pv->rx_cont;
        break;
#endif
      case SX127X_STATE_RX:
        rq = dev_rfpacket_rq_head(&pv->queue);
        break;
      case SX127X_STATE_RXC_STOP:
      case SX127X_STATE_RXC:
      case SX127X_STATE_RX_SCANNING:
        rq = pv->rx_cont;
        break;
      default:
        break;
    }

  assert(rq);

  /* Terminate allocated rx request */
  rx->size = size;
  rx->channel = pv->cfg.channel;
  rx->carrier = 0;
  rx->rssi = 0;
  rx->timestamp = pv->timestamp;
  rx->error = err;

  if (rq->anchor == DEV_RFPACKET_TIMESTAMP_START)
    rx->timestamp -= pv->rxrq->size * pv->timebit * 8;

  kroutine_exec(&rx->kr);
  pv->rxrq = NULL;
}

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE

static inline void sx127x_start_tx_raw(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

  assert(pv->state == SX127X_STATE_IDLE);
  assert(rq->pk_cfg->format == DEV_RFPACKET_FMT_RAW);

  uint32_t cmd = sx127x_set_cmd(pv, rq);

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_TX:
    case DEV_RFPACKET_RQ_TX_FAIR:
      sx127x_rfp_set_state(pv, SX127X_STATE_TX_RAW);
      sx127x_bytecode_start(pv, &sx127x_entry_tx_raw,
              SX127X_ENTRY_TX_RAW_BCARGS(cmd));
      break;
    default:
      abort();
  }
}

static void sx127x_rx_raw_timeout(struct sx127x_private_s *pv)
{
  switch (pv->state)
  {
    case SX127X_STATE_RX_RAW:
      /* Stop bitbang */
      if (DEVICE_OP(&pv->bitbang, cancel, &pv->brq))
        {
          sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW_PENDING_STOP);
          return;
        }
    default:
      break;
  }

  /* End allocated RX */
  sx127x_rfp_end_rxrq(pv, 0, 0);

  switch (pv->state)
  {
    case SX127X_STATE_RX_RAW:
    case SX127X_STATE_RX_RAW_ALLOC:
    case SX127X_STATE_RX_RAW_PENDING_STOP:
      break;
    default:
      abort();
  }

  sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW_STOP);
  sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
}

static KROUTINE_EXEC(sx127x_rx_raw_timeout_kr)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  struct sx127x_private_s *pv = sx127x_private_s_from_trq(trq);

  LOCK_SPIN_IRQ(&pv->dev->lock);
  sx127x_rx_raw_timeout(pv);
  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static KROUTINE_EXEC(sx127x_bitbang_tx_done)
{
  struct dev_bitbang_rq_s *rq = dev_bitbang_rq_from_kr(kr);
  struct sx127x_private_s *pv = sx127x_private_s_from_brq(rq);

  LOCK_SPIN_IRQ(&pv->dev->lock);

  assert(pv->state == SX127X_STATE_TX_RAW);
  assert(pv->bcrun == 0);

  /* Reset and configure device. Chip is freezed after a tx raw */
  sx127x_rfp_set_state(pv, SX127X_STATE_TX_RAW_DONE);
  sx127x_bytecode_start(pv, &sx127x_entry_standby, 0, 0);

  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static KROUTINE_EXEC(sx127x_bitbang_rx_done)
{
  struct dev_bitbang_rq_s *rq = dev_bitbang_rq_from_kr(kr);
  struct sx127x_private_s *pv = sx127x_private_s_from_brq(rq);

  LOCK_SPIN_IRQ(&pv->dev->lock);

  struct dev_bitbang_rq_s *rq = &pv->brq;
  
  /* End allocated RX */
  sx127x_rfp_end_rxrq(pv, rq->error, rq->count);

  switch (pv->state)
  {
    case SX127X_STATE_RXC_RAW_PENDING_STOP:
      sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_STOP);
      sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
      goto done;
    case SX127X_STATE_RXC_RAW:
      sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_ALLOC);
      break;
    case SX127X_STATE_RX_RAW_PENDING_STOP:
      sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW_STOP);
      sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
      goto done;
    case SX127X_STATE_RX_RAW:
      /* Cancel timer request */
      if (DEVICE_OP(pv->timer, cancel, &pv->trq))
        /* Kroutine will be called sooner */
        {
          sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW_PENDING_STOP);
          goto done;
        }
      sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW_ALLOC);
      break;
    default:
      abort();
  }

  /* Retry RX */
  kroutine_exec(&pv->kr);

done:
  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static void sx127x_start_tx_bitbang(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

  assert(rq && (rq->pk_cfg->format == DEV_RFPACKET_FMT_RAW));

  /* Start TX bitbang */ 

  pv->brq.type = DEV_BITBANG_WR;
  pv->brq.count = rq->tx_size;
  pv->brq.symbols = (void *)rq->tx_buf;

  dev_bitbang_rq_init(&pv->brq, sx127x_bitbang_tx_done);
  DEVICE_OP(&pv->bitbang, request, &pv->brq);
}

static void sx127x_start_rx_bitbang(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rx_s *rx = pv->rxrq;

  /* Start RX bitbang */ 

  pv->brq.type = DEV_BITBANG_RD;
  pv->brq.count = rx->size;
  pv->brq.symbols = rx->buf;

  dev_bitbang_rq_init(&pv->brq, sx127x_bitbang_rx_done);
  DEVICE_OP(&pv->bitbang, request, &pv->brq);
}

/* Transceiver is in RX when this function is called */
static void sx127x_rx_raw_startup(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rq_s *rq;
  dev_timer_value_t t;

  switch (pv->state)
  {
    case SX127X_STATE_RXC_RAW_ALLOC:
      rq = pv->rx_cont;
      break;
    case SX127X_STATE_RX_RAW_ALLOC:
      DEVICE_OP(pv->timer, get_value, &t, 0);
      /* Timeout date is already reached */
      if (t >= pv->timeout)
        return sx127x_rx_raw_timeout(pv);
      rq = dev_rfpacket_rq_head(&pv->queue);
      break;
    case SX127X_STATE_RXC_RAW_PENDING_STOP:
      sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_STOP);
      sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
      return;
    default:
      abort();
  }

  assert(rq);

  struct dev_rfpacket_pk_cfg_raw_s * cfg = (struct dev_rfpacket_pk_cfg_raw_s *)rq->pk_cfg;
  uint8_t size = cfg->mps;

  assert(size);

  pv->rxrq = rq->rx_alloc(rq, size);

  struct dev_rfpacket_rx_s *rx = pv->rxrq;

  if ((rx == NULL) || (rx->size != size))
    {
      /* Retry allocation later */
      kroutine_exec(&pv->kr);
      return;
    }
 
  /* Allocation success */

  sx127x_start_rx_bitbang(pv);

  switch (pv->state)
  {
    case SX127X_STATE_RXC_RAW_ALLOC:
      sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW);
      /* No need to start timer when RX continuous */
      return;
    case SX127X_STATE_RX_RAW_ALLOC:
      sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW);
      break;
    default:
      abort();
  }

  struct dev_timer_rq_s *trq = &pv->trq;

  trq->deadline = pv->timeout;
  trq->delay = 0;
  trq->rev = 0;
  trq->pvdata = pv;

  dev_timer_rq_init(trq, sx127x_rx_raw_timeout_kr);

  error_t err = DEVICE_OP(pv->timer, request, trq);

  switch (err)
    {
    case -ETIMEDOUT:
      sx127x_rx_raw_timeout(pv);
    case 0:
    default:
      break;
    }

  return;
}

static KROUTINE_EXEC(sx127x_retry_startup_kr)
{
  struct sx127x_private_s *pv = sx127x_private_s_from_kr(kr);

  LOCK_SPIN_IRQ(&pv->dev->lock);
  sx127x_rx_raw_startup(pv);
  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static void sx127x_rx_raw_end(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rq_s * rq;

  switch (pv->state)
  {
    case SX127X_STATE_RXC_RAW_STOP:
      rq = pv->rx_cont;
      rq->error = 0;
      /* RX continuous cancelled or replaced */ 
      if (pv->next_rx_cont != pv->rx_cont)
        dev_rfpacket_rq_done(rq);
      pv->rx_cont = pv->next_rx_cont;
      pv->next_rx_cont = NULL;
      return sx127x_rfp_idle(pv);
    case SX127X_STATE_RX_RAW_STOP:
      return sx127x_rfp_end_rq(pv, 0);
    default:
      abort();
  }
}

static void sx127x_cancel_rxc_raw(struct sx127x_private_s *pv)
{
  switch (pv->state)
    {
      case SX127X_STATE_RXC_RAW:
        sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_PENDING_STOP);
        /* Stop bitbang */
        if (DEVICE_OP(&pv->bitbang, cancel, &pv->brq))
          return;
        break;
      case SX127X_STATE_RXC_RAW_ALLOC:
        sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_PENDING_STOP);
      case SX127X_STATE_RXC_RAW_PENDING_STOP:
      case SX127X_STATE_RXC_RAW_STOP:
        return;
      default:
        abort();
    }
  
  /* End allocated RX packet */
  sx127x_rfp_end_rxrq(pv, 0, 0);

  sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_STOP);
  sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
}

static inline void sx127x_start_rx_raw(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  uint32_t cmd = sx127x_set_cmd(pv, rq);

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_RX:
      pv->timeout = t + rq->lifetime;
      sx127x_rfp_set_state(pv, SX127X_STATE_RX_RAW_ALLOC);
      break;
    case DEV_RFPACKET_RQ_RX_CONT:
      pv->rx_cont = rq;
      sx127x_rfp_set_state(pv, SX127X_STATE_RXC_RAW_ALLOC);
      break;
    default:
      abort();
  }

  sx127x_bytecode_start(pv, &sx127x_entry_rx_raw, SX127X_ENTRY_RX_RAW_BCARGS(cmd));
}
#endif

static void sx127x_retry_rx(struct sx127x_private_s *pv)
{
  assert(pv->state == SX127X_STATE_RX);

  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->queue);
  pv->cancel = 0;

  assert(rq && rq->type == DEV_RFPACKET_RQ_RX);

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  if (t >= pv->timeout)
    return sx127x_rfp_end_rq(pv, 0);

  uint32_t cmd = sx127x_set_cmd(pv, rq);

  sx127x_bytecode_start(pv, &sx127x_entry_rx,
    SX127X_ENTRY_RX_BCARGS(&pv->deadline, &pv->timeout, cmd));
}

static inline void sx127x_start_rx(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  assert(pv->state == SX127X_STATE_IDLE);
  assert(rq);

  pv->cancel = 0;

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_RAW)
    return sx127x_start_rx_raw(pv, rq);
#endif

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  uint32_t cmd = sx127x_set_cmd(pv, rq);

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_RX:
      dprintk("R\n");

      pv->deadline = rq->deadline ? rq->deadline : t;
      pv->timeout = pv->deadline + rq->lifetime;

      sx127x_rfp_set_state(pv, SX127X_STATE_RX);

      device_irq_src_enable(&pv->src_ep[1]);

      sx127x_bytecode_start(pv, &sx127x_entry_rx,
        SX127X_ENTRY_RX_BCARGS(&pv->deadline, &pv->timeout, cmd));
    break;

    case DEV_RFPACKET_RQ_RX_CONT:
      dprintk("RC\n");

      pv->rx_cont = rq;

      if (cmd & CMD_RX_SCANNING)
        sx127x_rfp_set_state(pv, SX127X_STATE_RX_SCANNING);
      else
        sx127x_rfp_set_state(pv, SX127X_STATE_RXC);

      sx127x_bytecode_start(pv, &sx127x_entry_rx_cont, 
         SX127X_ENTRY_RX_CONT_BCARGS(cmd, pv->tpbrx));
      break;

    default:
      abort();
  }
}

static void sx127x_start_tx(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

  assert(rq);
  assert(pv->state == SX127X_STATE_IDLE);

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
  if (rq->pk_cfg->format == DEV_RFPACKET_FMT_RAW)
    return sx127x_start_tx_raw(pv);
#endif

  assert(rq->tx_size <= CONFIG_DRIVER_RFPACKET_SX127X_MAX_PKT_SIZE);

  /* Get timer value */
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  pv->deadline = rq->deadline ? rq->deadline : t;

  uint32_t cmd = sx127x_set_cmd(pv, rq);

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_TX_FAIR:
      pv->timeout = pv->deadline + rq->lifetime;

      if (t >= pv->timeout)
        /* Timeout date is already reached */
        return sx127x_rfp_end_rq(pv, -ETIMEDOUT);

      sx127x_rfp_set_state(pv, SX127X_STATE_TX_FAIR);

      sx127x_bytecode_start(pv, &sx127x_entry_tx_fair,
              SX127X_ENTRY_TX_FAIR_BCARGS(rq->tx_size, rq->tx_buf, cmd, &pv->deadline,
                                     &pv->timeout, -pv->cfg.rssi_th));
      break;
    case DEV_RFPACKET_RQ_TX:

      sx127x_rfp_set_state(pv, SX127X_STATE_TX);

      sx127x_bytecode_start(pv, &sx127x_entry_tx,
              SX127X_ENTRY_TX_BCARGS(rq->tx_size, rq->tx_buf, cmd, &pv->deadline));
      break;

    default:
      abort();
  }
}

static void sx127x_rfp_end_rxc(struct sx127x_private_s *pv, error_t err)
{
  struct dev_rfpacket_rq_s * rq = pv->rx_cont;

  assert(rq);

  rq->error = err;

  if (pv->next_rx_cont != pv->rx_cont)
    {
      dev_rfpacket_rq_done(pv->rx_cont);
      pv->rx_cont = pv->next_rx_cont;
    }

  pv->next_rx_cont = NULL;

  return sx127x_rfp_idle(pv);
}

static void sx127x_rfp_end_rq(struct sx127x_private_s *pv, error_t err)
{
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->queue);

  assert(rq && rq->type != DEV_RFPACKET_RQ_RX_CONT);

  rq->error = err;
  rq->base.drvdata = NULL;

  dev_rfpacket_rq_pop(&pv->queue);
  dev_rfpacket_rq_done(rq);

  device_irq_src_disable(&pv->src_ep[1]);

  if (rq->error)
    sx127x_rfp_process_group(pv, rq->err_group);

  return sx127x_rfp_idle(pv);
}

static void sx127x_rfp_idle(struct sx127x_private_s *pv)
{
  assert(pv->bcrun == 0);

  sx127x_rfp_set_state(pv, SX127X_STATE_IDLE);

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

  if (!rq)
    rq = pv->rx_cont;

  if (!rq)
      return;


  dprintk("idle %d\n", rq->type);

  /* Check transceiver configuration */
  switch (sx127x_check_config(pv, rq))
    {
    case -EAGAIN:
      /** Configuration is being applied */
      sx127x_rfp_set_state(pv, SX127X_STATE_CONFIG);
      return;
    case -ENOTSUP:
      /** Unsupported configuration */
      if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
        return sx127x_rfp_end_rxc(pv, -ENOTSUP);
      else
        return sx127x_rfp_end_rq(pv, -ENOTSUP);
    default:
      /* Configuration is already applied */
      break;
    }

  switch (rq->type)
  {
    case DEV_RFPACKET_RQ_TX_FAIR:
    case DEV_RFPACKET_RQ_TX:
      return sx127x_start_tx(pv);
    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
      return sx127x_start_rx(pv, rq);
    default:
      return sx127x_rfp_end_rq(pv, -ENOTSUP);
  }
}

static DEV_RFPACKET_REQUEST(sx127x_request)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  va_list vl;

  LOCK_SPIN_IRQ(&dev->lock);

  va_start(vl, accessor);

  while (1)
    {
      struct dev_rfpacket_rq_s * rq = va_arg(vl, struct dev_rfpacket_rq_s *);

      if (!rq)
        break;

      assert(rq != pv->rx_cont);
      assert(rq != pv->next_rx_cont);

      dprintk("req %d %d %d\n", rq->type, rq->tx_size, pv->state);

      if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
        {

          switch (pv->state)
          {
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
            case SX127X_STATE_RXC_RAW_PENDING_STOP:
            case SX127X_STATE_RXC_RAW_STOP:
              if (pv->next_rx_cont && pv->next_rx_cont != pv->rx_cont)
                dev_rfpacket_rq_done(pv->next_rx_cont);
            case SX127X_STATE_RXC_RAW_ALLOC:
            case SX127X_STATE_RXC_RAW:
              assert(pv->rx_cont);
              sx127x_cancel_rxc_raw(pv);
              pv->next_rx_cont = rq;
              break;
#endif
            case SX127X_STATE_RXC:
            case SX127X_STATE_RX_SCANNING:
              assert(pv->rx_cont);
              sx127x_cancel_rxc(pv);
              pv->next_rx_cont = rq;
              break;
            case SX127X_STATE_RXC_PENDING_STOP:
            case SX127X_STATE_RXC_STOP:
              assert(pv->rx_cont);
              if (pv->next_rx_cont && pv->next_rx_cont != pv->rx_cont)
                dev_rfpacket_rq_done(pv->next_rx_cont);
              pv->next_rx_cont = rq;
              break;
            case SX127X_STATE_IDLE:
              assert(pv->rx_cont == NULL);
              pv->rx_cont = rq;
              sx127x_rfp_idle(pv);
              break;
            default:
              assert(pv->next_rx_cont == NULL);
              if (pv->rx_cont)
                dev_rfpacket_rq_done(pv->rx_cont);
              pv->rx_cont = rq;
              break;
          }
      }
    else
      {
        bool_t empty = dev_rq_queue_isempty(&pv->queue);
        dev_rfpacket_rq_pushback(&pv->queue, rq);
        rq->base.drvdata = pv;

        if (empty)
          {
            switch (pv->state)
            {
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
              case SX127X_STATE_RXC_RAW:
              case SX127X_STATE_RXC_RAW_ALLOC:
                assert(pv->rx_cont);
                assert(rq->deadline == 0);
                assert(pv->next_rx_cont == NULL);
                sx127x_cancel_rxc_raw(pv);
                /* The next rx continous is the same as the current one */
                pv->next_rx_cont = pv->rx_cont;
                break;
#endif
              case SX127X_STATE_RXC:
              case SX127X_STATE_RX_SCANNING:
                assert(pv->rx_cont);
                assert(rq->deadline == 0);
                assert(pv->next_rx_cont == NULL);
                /* The next rx continous is the same as the current one */
                pv->next_rx_cont = pv->rx_cont;
                sx127x_cancel_rxc(pv);
                break;
              case SX127X_STATE_IDLE:
                sx127x_rfp_idle(pv);
                break;
              default:
                break;
            }
          }
      }
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}


/* Transceiver is idle when this function is called */
static inline void sx127x_rx_done(struct sx127x_private_s *pv)
{
  sx127x_rfp_end_rxrq(pv, 0, pv->size);

  switch (pv->state)
  {
    case SX127X_STATE_RX:
      if (!(pv->bc_status & _MSK(STATUS_RX_PAYLOAD_RDY)))
      /* Lifetime is up */
        return sx127x_rfp_end_rq(pv, 0);
      /* A packet has been received */
      return sx127x_retry_rx(pv);
    case SX127X_STATE_RXC_PENDING_STOP:
    case SX127X_STATE_RXC_STOP:
      return sx127x_rfp_end_rxc(pv, 0);
    case SX127X_STATE_RXC:
    case SX127X_STATE_RX_SCANNING:
      assert(pv->next_rx_cont == NULL);
      return sx127x_rfp_idle(pv);
    default:
      abort();
  }
}

static inline void sx127x_tx_done(struct sx127x_private_s *pv)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  dprintk("TX irq\n");

  assert(rq);

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_STATS
  pv->stats.tx_count++;
#endif

  switch (pv->state)
  {
    case SX127X_STATE_TX_FAIR:
      if (!(pv->bc_status & _MSK(STATUS_TX_SENT)))
        {
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_STATS
          pv->stats.tx_err_count++;
#endif
          /* Lifetime is up */
          return sx127x_rfp_end_rq(pv, -ETIMEDOUT);
        }
    case SX127X_STATE_TX:
      /* Packet has been transmitted */
      rq->tx_timestamp = pv->timestamp;
      if (rq->anchor == DEV_RFPACKET_TIMESTAMP_START)
        rq->tx_timestamp -= rq->tx_size * pv->timebit * 8;
      return sx127x_rfp_end_rq(pv, 0);
    default:
      abort();
  }
}

static DEV_IRQ_SRC_PROCESS(sx127x_irq_process)
{
  struct device_s *dev = ep->base.dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  lock_spin(&dev->lock);

  pv->icount++;
  /* Get timer value */
  DEVICE_OP(pv->timer, get_value, &pv->timestamp, 0);
  /* Wakeup any waiting instruction */
  dev_spi_bytecode_wakeup(&pv->spi, srq);
  /* Try to enter bytecode if not running */
  if (!pv->bcrun)
    sx127x_bytecode_start(pv, &sx127x_entry_irq, 0, 0);

  lock_release(&dev->lock);
}

static void sx127x_cancel_rxc(struct sx127x_private_s *pv)
{
  switch (pv->state)
    {
      /* Rx continous. No loop in bytecode */
      case SX127X_STATE_RXC:
        if (pv->bcrun)
          {
             sx127x_rfp_set_state(pv, SX127X_STATE_RXC_PENDING_STOP);
             return;
          }
        sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
      /* Rx continous. Loop in bytecode */
      case SX127X_STATE_RX_SCANNING:
        pv->cancel = 1;
        sx127x_rfp_set_state(pv, SX127X_STATE_RXC_STOP);
        break;
      default:
        abort();
    }
}

static DEV_RFPACKET_CANCEL(sx127x_cancel)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  error_t err = -EBUSY;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  struct dev_rfpacket_rq_s *hrq = dev_rfpacket_rq_head(&pv->queue);

  if (rq == pv->rx_cont)
    {
      switch (pv->state)
      {
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
        case SX127X_STATE_RXC_RAW:
        case SX127X_STATE_RXC_RAW_ALLOC:
          assert(pv->next_rx_cont == NULL);
        case SX127X_STATE_RXC_RAW_PENDING_STOP:
        case SX127X_STATE_RXC_RAW_STOP:
          sx127x_cancel_rxc_raw(pv);
          break;
#endif
        case SX127X_STATE_RXC_STOP:
          if (rq == pv->next_rx_cont)
            pv->next_rx_cont = NULL;
          break;
        case SX127X_STATE_RXC:
        case SX127X_STATE_RX_SCANNING:
          assert(pv->next_rx_cont == NULL);
          sx127x_cancel_rxc(pv);
          break;
        default:
          err = 0;
        pv->rx_cont = NULL;
        if (rq == pv->next_rx_cont)
            pv->next_rx_cont = NULL;
          break;
      }
    }
  else if(rq == pv->next_rx_cont)
    {
      switch (pv->state)
      {
        case SX127X_STATE_RXC_STOP:
          pv->next_rx_cont = NULL;
          if (rq != pv->rx_cont)
            err = 0;
          break;

        default:
          abort();
      }
    }
  else if ((rq->base.drvdata == pv) && (rq != hrq))
  /* Request is in queue and is not being processed */
    {
      err = 0;
      rq->base.drvdata = NULL;
      dev_rfpacket_rq_remove(&pv->queue, rq);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_RFPACKET_STATS(sx127x_stats)
{
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_STATS
  struct device_s *dev = accessor->dev;
  struct sx127x_private_s *pv = dev->drv_pv;
  memcpy(stats, &pv->stats, sizeof(*stats));
  return 0;
#else
  return -ENOTSUP;
#endif
}

/* ******************************** TIMER ************************************/

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_TIMER)

static
DEV_TIMER_REQUEST(sx127x_timer_request)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  err = DEVICE_OP(pv->timer, request, rq);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_TIMER_CANCEL(sx127x_timer_cancel)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  err = DEVICE_OP(pv->timer, cancel, rq);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_TIMER_GET_VALUE(sx127x_timer_get_value)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  err = DEVICE_OP(pv->timer, get_value, value, rev);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_TIMER_CONFIG(sx127x_timer_config)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  err = DEVICE_OP(pv->timer, config, cfg, res);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#endif

static void sx127x_clean(struct device_s *dev)
{
  struct sx127x_private_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, pv->src_ep, 2);

  device_stop(&pv->timer->base);

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);

  mem_free(pv);
}

/* Transceiver is idle when this function is called */
static inline void sx127x_rfp_error(struct sx127x_private_s *pv)
{
  dprintk("sx127x: -EIO error %d\n", pv->state);
  /* Terminate allocated rx request */
  sx127x_rfp_end_rxrq(pv, -EIO, 0);

  switch (pv->state)
  {
    case SX127X_STATE_TX:
    case SX127X_STATE_TX_FAIR:
      return sx127x_rfp_end_rq(pv, -EIO);
    case SX127X_STATE_RX:
      return sx127x_retry_rx(pv);
    case SX127X_STATE_RXC_PENDING_STOP:
    case SX127X_STATE_RXC_STOP:
      return sx127x_rfp_end_rxc(pv, 0);
    case SX127X_STATE_RXC:
    case SX127X_STATE_RX_SCANNING:
      return sx127x_rfp_idle(pv);
    default:
      abort();
  }
}

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_PRINT_REGS
static void sx127x_print_registers(struct sx127x_private_s *pv)
{
  for (uint32_t i = 0; i < 112; i++)
    dprintk(" 0x%02x: 0x%x\n", i+1, pv->dump[i]);
}
#endif

static KROUTINE_EXEC(sx127x_spi_rq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->pvdata;
  struct sx127x_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  pv->bcrun = 0;
  pv->bc_status = bc_get_reg(&srq->vm, STATUS);

  dprintk("bdone %d 0x%x %x \n", pv->state, pv->bc_status, pv->icount);

  if (pv->bc_status & _MSK(STATUS_OTHER_ERR))
    {
      sx127x_rfp_error(pv);
      goto end;
    }

  if (pv->bc_status & _MSK(STATUS_RX_PAYLOAD_RDY))
    {
      sx127x_rx_done(pv);
      goto end;
    }

  switch (pv->state)
  {
    case SX127X_STATE_INITIALISING:
      if (srq->error)
        {
          sx127x_clean(dev);
          device_async_init_done(dev, -EIO);
          goto end;
        }
      device_async_init_done(dev, 0);
    case SX127X_STATE_CONFIG:
      sx127x_rfp_idle(pv);
      break;
    case SX127X_STATE_RX:
      sx127x_rx_done(pv);
      break;
    case SX127X_STATE_TX_FAIR:
    case SX127X_STATE_TX:
      sx127x_tx_done(pv);
      break;
    case SX127X_STATE_RXC:
    case SX127X_STATE_RX_SCANNING:
      break;
    case SX127X_STATE_RXC_PENDING_STOP:
      sx127x_rfp_set_state(pv, SX127X_STATE_RXC_STOP);
      sx127x_bytecode_start(pv, &sx127x_entry_cancel, 0, 0);
      break;  
    case SX127X_STATE_RXC_STOP:
      sx127x_rfp_end_rxc(pv, 0);
      break;
#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
    case SX127X_STATE_TX_RAW_DONE:
      sx127x_rfp_end_rq(pv, 0);
      break;
    case SX127X_STATE_TX_RAW:
      sx127x_start_tx_bitbang(pv);
      break;
    case SX127X_STATE_RXC_RAW_PENDING_STOP:
    case SX127X_STATE_RXC_RAW_ALLOC:
    case SX127X_STATE_RX_RAW:
    case SX127X_STATE_RX_RAW_ALLOC:
      sx127x_rx_raw_startup(pv);
      break;
    case SX127X_STATE_RXC_RAW_STOP:
    case SX127X_STATE_RX_RAW_STOP:
      sx127x_rx_raw_end(pv);
      break;
#endif
    default:
      abort();
  }

  if (!pv->bcrun && (pv->icount != (pv->bc_status & STATUS_IRQ_MSK)))
    sx127x_bytecode_start(pv, &sx127x_entry_irq, 0, 0);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

/* ******************************** Device init/cleanup **********************/

static DEV_INIT(sx127x_init)
{
  struct sx127x_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->dev = dev;

#ifdef SX127X_PRINT_REGS
  pv->dump = mem_alloc(113, (mem_scope_sys));
  if (!pv->dump) 
    return -ENOMEM;
#endif

  struct dev_freq_s osc;
  if (!device_get_res_freq(dev, &osc, 0))
    pv->osc_ppb = dev_freq_acc_ppb(&osc);
  else
    pv->osc_ppb = 20000;

  struct dev_spi_ctrl_bytecode_rq_s * srq  = &pv->spi_rq;
  struct device_gpio_s              * gpio = NULL;

  static struct dev_spi_ctrl_config_s const spi_ctrl_cfg =
    {
      .bit_rate1k = CONFIG_DRIVER_RFPACKET_SX127X_SPI_BITRATE >> 10,
      .word_width = 8,
      .bit_order  = DEV_SPI_MSB_FIRST,
      .miso_pol   = DEV_SPI_ACTIVE_HIGH,
      .mosi_pol   = DEV_SPI_ACTIVE_HIGH,
      .cs_pol     = DEV_SPI_ACTIVE_LOW,
      .ck_mode    = DEV_SPI_CK_MODE_0,
    };

  if (dev_drv_spi_bytecode_init(dev, srq, &sx127x_bytecode, &spi_ctrl_cfg,
                                &pv->spi, &gpio, &pv->timer))
    goto err_mem;

#if defined(CONFIG_DEVICE_SPI_BYTECODE_TIMER)
  /* Base 1 ms time */
  dev_timer_init_sec(pv->timer, &pv->delay_1ms, 0, 1, 1000);
  /* Start timer */
  if (device_start(&pv->timer->base))
    goto err_srq;
#else
    goto err_srq;
#endif

  srq->base.cs_cfg.polarity = DEV_SPI_ACTIVE_LOW;
  srq->pvdata = dev;

  /* Init GPIO stuff */

  static const gpio_width_t pin_wmap[SX127X_PIN_COUNT] = {1, 1, 1};

  if (device_gpio_setup(gpio, dev, ">rst:1 <dio0:1 <dio4:1", pv->pin_map, NULL))
    goto err_timer;

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

#ifdef CONFIG_DRIVER_RFPACKET_SX127X_RAW_MODE
  if (device_get_param_dev_accessor(dev, "bitbang", &pv->bitbang.base, DRIVER_CLASS_BITBANG))
    goto err_timer;

  kroutine_init_deferred(&pv->kr, &sx127x_retry_startup_kr);
#endif

  dev_rq_queue_init(&pv->queue);

  dev_spi_ctrl_rq_init(&srq->base, &sx127x_spi_rq_done);

  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  /* irq */
  device_irq_source_init(dev, pv->src_ep, 2, &sx127x_irq_process);

  if (device_irq_source_link(dev, pv->src_ep, 2, -1))
    goto err_timer;

  /* Disable DIO4 as irq */
  device_irq_src_disable(&pv->src_ep[1]);

  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  sx127x_bytecode_start(pv, &sx127x_entry_reset, 0);

  pv->state = SX127X_STATE_INITIALISING;

  dev->drv_pv = pv;

  return -EAGAIN;

err_link:
  device_irq_source_unlink(dev, pv->src_ep, 1);
err_timer:
  device_stop(&pv->timer->base);
err_srq:
  dev_drv_spi_bytecode_cleanup(&pv->spi, srq);
err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(sx127x_cleanup)
{
  struct sx127x_private_s *pv = dev->drv_pv;

  switch (pv->state)
    {
    case SX127X_STATE_IDLE:
      assert(dev_rq_queue_isempty(&pv->queue));
      break;
    default:
      return -EBUSY;
    }

  sx127x_clean(dev);
  return 0;
}

#define sx127x_use      dev_use_generic

DRIVER_DECLARE(sx127x_drv, 0, "Semtech SX127x Transceiver", sx127x,
               DRIVER_RFPACKET_METHODS(sx127x)
#if defined(CONFIG_DRIVER_RFPACKET_SX127X_TIMER)
              ,DRIVER_TIMER_METHODS(sx127x_timer)
#endif
#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
              ,DRIVER_CRYPTO_METHODS(sx127x_crypto)
#endif
              );

DRIVER_REGISTER(sx127x_drv);
