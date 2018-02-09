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
      - does not support continous mode
      - does not support packet size greater than 64 bytes.
      - does not support packet reception during LBT.
      - does not use PA_BOOST pin for TX.
      - no cache management.

*/

#include "sx127x_spi.h"
#include "sx127x_spi.o.h"

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
  dprintk("state %d\n", state);
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

static inline void sx127x_config_freq_update(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s * rq, uint32_t channel)
{
  uint32_t const freq = (pv->cfg.coeff.freq + channel * pv->cfg.coeff.chan) >> 15;
  uint8_t *f = pv->cfg.freq;

  f[0] = freq >> 16;
  f[1] = freq >> 8;
  f[2] = freq;

  pv->cfg.channel = channel;
}

static inline void sx127x_config_coeff_freq_update(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s * rq)
{
  pv->cfg.coeff.freq = ((uint64_t)rq->rf_cfg->frequency << (19 + 15)) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;
  pv->cfg.coeff.chan = ((uint64_t)rq->rf_cfg->chan_spacing << (19 + 15)) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;

  sx127x_config_freq_update(pv, rq, rq->channel);
}

struct sx127x_config_bw_s
{
  uint32_t BITFIELD(bw, 24);
  uint8_t  bits;
};

static const struct sx127x_config_bw_s sx127x_config_bw_tbl[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    /* Invalid bandwidth */
    { 300000, 0x00 }, 
};

static uint8_t sx127x_get_bw(uint32_t bw)
{
  for (uint16_t i = 0; i < ARRAY_SIZE(sx127x_config_bw_tbl); i++)
    {
      if ((bw >= sx127x_config_bw_tbl[i].bw) && 
        (bw < sx127x_config_bw_tbl[i + 1].bw))

        return sx127x_config_bw_tbl[i].bits;
    }

  return 0;
}

static error_t sx127x_build_raw_config(struct sx127x_private_s * pv, struct dev_rfpacket_rq_s *rq, uint8_t **p)
{
  uint8_t * pk = *p;

  /* No AFC and no AGC */

  pv->cfg.rxcfg = 0;

  *pk++ = 1;
  *pk++ = SX1276_REG_RXCONFIG | 0x80;
  *pk++ = pv->cfg.rxcfg;

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

static error_t sx127x_build_pkt_config(struct sx127x_private_s * pv, struct dev_rfpacket_rq_s *rq, uint8_t **p)
{
  uint8_t * pk = *p;
  const struct dev_rfpacket_pk_cfg_basic_s *pk_cfg = const_dev_rfpacket_pk_cfg_basic_s_cast(rq->pk_cfg);

  /* Fifo threshold */

  *pk++ = 1;
  *pk++ = SX1276_REG_FIFOTHRESH | 0x80;
  *pk++ = 0x80 | 0x3F;

  /* LNA gain and RXConfig */

  pv->cfg.rxcfg = SX1276_RXCONFIG_AFCAUTO_ON |
                  SX1276_RXCONFIG_AGCAUTO_ON |
                  SX1276_RXCONFIG_RXTRIGER_PREAMBLEDETECT;
  
  *pk++ = 2;
  *pk++ = SX1276_REG_LNA | 0x80;
  *pk++ = 0x00;
  *pk++ = pv->cfg.rxcfg;

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

      x = ((x & 0x55) << 1) | ((0xAA & x) >> 1);
      x = ((x & 0x33) << 2) | ((0xCC & x) >> 2);
      x = ((x & 0x0F) << 4) | ((0xF0 & x) >> 4);

      *pk++ = x;
    }

  if (pk_cfg->rx_pb_len > 24)
    return -ENOTSUP;

  /*  Sync word + preambule + crc + fifo filling time + 2 bytes margin */
  pv->tbrs = pv->timebit * ((68 + sw) * 8 + IO_MAX_PREAMBLE_SIZE);

  /* Time to achieve preambule detection */
  dev_timer_delay_t t = SX127X_TS_RE_US + SX127X_TS_FS_US + 100;
  dev_timer_init_sec(pv->timer, &pv->tpbrx, 0, t, 1000000);
  pv->tpbrx += pv->timebit * (pk_cfg->rx_pb_len >> 3) * 8;

  *pk++ = 1;
  *pk++ = SX1276_REG_PREAMBLEDETECT | 0x80;
  *pk++ = SX1276_PREAMBLEDETECT_DETECTOR_ON | (((pk_cfg->rx_pb_len >> 3) - 1) << 5) | 0xA;

  /* Packet config mode */

  *pk++ = 3;
  *pk++ = SX1276_REG_PACKETCONFIG1 | 0x80;

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

  switch (rq->pk_cfg->encoding)
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

static error_t sx127x_build_fsk_config(struct sx127x_private_s * pv, struct dev_rfpacket_rq_s *rq, uint8_t **f)
{
  const struct dev_rfpacket_rf_cfg_fsk_s * fsk = const_dev_rfpacket_rf_cfg_fsk_s_cast(rq->rf_cfg);

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
    *rf++ = 0x09 | SX1276_PARAMP_MODULATIONSHAPING_01;
  else 
    *rf++ = 0x09 | SX1276_PARAMP_MODULATIONSHAPING_00;

  *f = rf;

  pv->cfg.mode = SX1276_OPMODE_MODULATIONTYPE_FSK |
                 SX1276_OPMODE_LONGRANGEMODE_OFF;

  return 0;
}

static error_t sx127x_build_ask_config(struct sx127x_private_s * pv, struct dev_rfpacket_rq_s *rq, uint8_t **f)
{
  const struct dev_rfpacket_rf_cfg_ask_s * ask = const_dev_rfpacket_rf_cfg_ask_s_cast(rq->rf_cfg);
  
  if (ask->symbols > 2)
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
  *rf++ = 0x09;

  *f = rf;

  pv->cfg.mode = SX1276_OPMODE_MODULATIONTYPE_OOK |
                 SX1276_OPMODE_LONGRANGEMODE_OFF;

  return 0;
}

static inline error_t sx127x_check_config(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
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

  if ((rfcfg != pv->rf_cfg) || rfcfg->cache.dirty)
  /* Test if new RF configuration or previous configuration modified */
    {
      pv->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)rfcfg;

      /* Compute time to send a bit in us */
      dev_timer_delay_t t = 1000000/rq->rf_cfg->drate;
      dev_timer_init_sec(pv->timer, &pv->timebit, 0, t, 1000000);

      if (rfcfg->cache.dirty)
        ((struct dev_rfpacket_rf_cfg_s *)rfcfg)->cache.dirty = 0;

      /* Frequency */
      sx127x_config_coeff_freq_update(pv, rq);

      *cfg++ = 2;
      *cfg++ = SX1276_REG_BITRATEMSB | 0x80;
 
      /* Datarate */
      uint16_t dr = 0x01;
      if (rq->pk_cfg->format != DEV_RFPACKET_FMT_RAW)
        dr = (CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO/rfcfg->drate);

      endian_be16_na_store(cfg, dr);
      cfg += 2;

      /* Bandwidth */
      *cfg++ = 2;
      *cfg++ = SX1276_REG_RXBW | 0x80;
  
      uint8_t bw = sx127x_get_bw(rfcfg->bw);

      if (!bw)
        return -EINVAL;
  
      uint8_t afcbw = sx127x_get_bw(rfcfg->bw);
 
      if (!afcbw)
        return -EINVAL;
  
      *cfg++ = bw;
      *cfg++ = afcbw;

      switch (rfcfg->mod)
        {
          case DEV_RFPACKET_FSK: 
          case DEV_RFPACKET_GFSK: 
            if (sx127x_build_fsk_config(pv, rq, &cfg))
              return -ENOTSUP;
            break;
          case DEV_RFPACKET_ASK: 
            if (sx127x_build_ask_config(pv, rq, &cfg))
              return -ENOTSUP;
            break;
          default:
            return -ENOTSUP;
        }

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
          case DEV_RFPACKET_FMT_RAW:
            if (rfcfg->mod != DEV_RFPACKET_ASK)
              return -ENOTSUP;
            if (sx127x_build_raw_config(pv, rq, &cfg))
              return -ENOTSUP;
            break;
          default:
            return -ENOTSUP;
        }
     
      change = 1;
    }

  if (!change)
    return 0;
  
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
    rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

    if (!rq || rq->err_group != group)
      break;

    assert(rq->type != DEV_RFPACKET_RQ_RX_CONT);

    rq->err = -ECANCELED;
    dev_request_queue_pop(&pv->queue);
    kroutine_exec(&rq->base.kr);
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

  uint32_t mask = pv->rx_cont->mask;

  uint32_t tmp = ~((1 << (pv->cfg.channel + 1)) - 1) & mask;
  uint8_t next = tmp ? bit_ctz(tmp) : bit_ctz(mask);

  sx127x_config_freq_update(pv, pv->rx_cont, next);
  
  bc_set_reg(ctx, 0, next);

  LOCK_RELEASE_IRQ(&pv->dev->lock);
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
      rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));
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
      size = (pv->size &  ((1 << 5) - 1)) + 1;
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
  bc_set_reg(ctx, 0, (uintptr_t)p);
}

static uint32_t sx127x_set_cmd(struct sx127x_private_s *pv, struct dev_rfpacket_rq_s *rq)
{
  uint32_t cmd = pv->cfg.sync;

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
        if (rq->mask != 1)
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
  sx127x_config_freq_update(pv, rq, rq->channel);
  
  return cmd;
}


static void sx127x_retry_rx(struct sx127x_private_s *pv)
{
  assert(pv->state == SX127X_STATE_RX);

  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));
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
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

  assert(rq && rq->tx_size < CONFIG_DRIVER_RFPACKET_SX127X_MAX_PKT_SIZE);
  assert(pv->state == SX127X_STATE_IDLE);

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

  if (rq)
    rq->err = err;

  switch (pv->state)
  {
    case SX127X_STATE_CONFIG_RXC:
      assert(pv->next_rx_cont == NULL);
    case SX127X_STATE_CONFIG_RXC_PENDING_STOP:
      assert(rq && rq != pv->next_rx_cont);
      kroutine_exec(&rq->base.kr);
      pv->rx_cont = pv->next_rx_cont;
      pv->next_rx_cont = NULL;
      return sx127x_rfp_idle(pv);
    case SX127X_STATE_RXC_STOP:
      assert(rq);
      if (pv->next_rx_cont != pv->rx_cont)
        {
          kroutine_exec(&pv->rx_cont->base.kr);
          pv->rx_cont = pv->next_rx_cont;
        }
      pv->next_rx_cont = NULL;
      return sx127x_rfp_idle(pv);
    default:
      abort();
  }
}

static void sx127x_rfp_end_rq(struct sx127x_private_s *pv, error_t err)
{
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

  assert(rq && rq->type != DEV_RFPACKET_RQ_RX_CONT);

  rq->err = err;

  dev_request_queue_pop(&pv->queue);
  kroutine_exec(&rq->base.kr);

  if (rq->err)
    sx127x_rfp_process_group(pv, rq->err_group);

  return sx127x_rfp_idle(pv);
}

static void sx127x_rfp_idle(struct sx127x_private_s *pv)
{
  assert(pv->bcrun == 0);

  sx127x_rfp_set_state(pv, SX127X_STATE_IDLE);

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

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
      if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
        sx127x_rfp_set_state(pv, SX127X_STATE_CONFIG_RXC);
      else
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
          rq->base.drvdata = NULL;

          switch (pv->state)
          {
            case SX127X_STATE_RXC:
            case SX127X_STATE_RX_SCANNING:
              assert(pv->rx_cont);
              sx127x_cancel_rxc(pv);
              pv->next_rx_cont = rq;
              break;
            case SX127X_STATE_CONFIG_RXC_PENDING_STOP:
              if (pv->next_rx_cont)
                kroutine_exec(&pv->next_rx_cont->base.kr);
            case SX127X_STATE_CONFIG_RXC:
              assert(pv->rx_cont);
              pv->next_rx_cont = rq;
              sx127x_rfp_set_state(pv, SX127X_STATE_CONFIG_RXC_PENDING_STOP);
              break;
            case SX127X_STATE_RXC_PENDING_STOP:
            case SX127X_STATE_RXC_STOP:
              assert(pv->rx_cont);
              if (pv->next_rx_cont && pv->next_rx_cont != pv->rx_cont)
                kroutine_exec(&pv->next_rx_cont->base.kr);
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
                kroutine_exec(&pv->rx_cont->base.kr);
              pv->rx_cont = rq;
              break;
          }
      }
    else
      {
        bool_t empty = dev_request_queue_isempty(&pv->queue);
        dev_request_queue_pushback(&pv->queue, dev_rfpacket_rq_s_base(rq));
        rq->base.drvdata = pv;

        if (empty)
          {
            switch (pv->state)
            {
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

static void sx127x_rfp_end_rxrq(struct sx127x_private_s *pv, bool_t err)
{
  struct dev_rfpacket_rx_s *rx = pv->rxrq;

  if (rx == NULL)
    return;

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

  /* Terminate allocated rx request */
  if (err)
    rx->size = 0;
  else
    {
      switch (pv->state)
      {
        case SX127X_STATE_RX:
          rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));
          break;
        case SX127X_STATE_RXC_STOP:
        case SX127X_STATE_RXC:
        case SX127X_STATE_RX_SCANNING:
          rq = pv->rx_cont;
          break;
        default:
          abort();
      }

      assert(rq);

      rx->channel = pv->cfg.channel;
      rx->carrier = 0;
      rx->rssi = 0;
      rx->timestamp = pv->timestamp;

      if (rq->anchor == DEV_RFPACKET_TIMESTAMP_START)
        rx->timestamp -= pv->rxrq->size * pv->timebit * 8;
    }

  rx->err = err;
  kroutine_exec(&rx->kr);
  pv->rxrq = NULL;
}

/* Transceiver is idle when this function is called */
static inline void sx127x_rx_done(struct sx127x_private_s *pv)
{
  sx127x_rfp_end_rxrq(pv, 0);

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
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));
  dprintk("TX irq\n");

  assert(rq);

  switch (pv->state)
  {
    case SX127X_STATE_TX_FAIR:
      if (!(pv->bc_status & _MSK(STATUS_TX_SENT)))
      /* Lifetime is up */
        return sx127x_rfp_end_rq(pv, -ETIMEDOUT);
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

  switch (ep - pv->src_ep)
  {
    case 0:
      pv->icount++;
      /* Get timer value */
      DEVICE_OP(pv->timer, get_value, &pv->timestamp, 0);
      /* Wakeup any waiting instruction */
      dev_spi_bytecode_wakeup(&pv->spi, srq);
      /* Try to enter bytecode if not running */
      if (!pv->bcrun)
        sx127x_bytecode_start(pv, &sx127x_entry_irq, 0, 0);
      break;
  }

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

  dprintk("cancel %d\n", pv->state);

  error_t err = -EBUSY;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq == pv->rx_cont)
    {
      switch (pv->state)
      {
        case SX127X_STATE_CONFIG_RXC:
          assert(pv->next_rx_cont == NULL);
          sx127x_rfp_set_state(pv, SX127X_STATE_CONFIG_RXC_PENDING_STOP);
          break;
        case SX127X_STATE_RXC_STOP:
          if (rq == pv->next_rx_cont)
            pv->next_rx_cont = NULL;
          break;
        case SX127X_STATE_CONFIG_RXC_PENDING_STOP:
          assert(rq != pv->next_rx_cont);
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
        case SX127X_STATE_CONFIG_RXC_PENDING_STOP:
        case SX127X_STATE_RXC_STOP:
          pv->next_rx_cont = NULL;
          if (rq != pv->rx_cont)
            err = 0;
          break;

        default:
          abort();
      }
    }
  else if (rq->base.drvdata == pv)
    {
      err = 0;
      rq->base.drvdata = NULL;
      dev_request_queue_remove(&pv->queue, dev_rfpacket_rq_s_base(rq));
    }


  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_RFPACKET_STATS(sx127x_stats)
{
  return -ENOTSUP;
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
  sx127x_rfp_end_rxrq(pv, 1);

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
static KROUTINE_EXEC(sx127x_spi_rq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->base.base.pvdata;
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
      if (srq->base.err)
        {
          sx127x_clean(dev);
          device_async_init_done(dev, -EIO);
          goto end;
        }
      device_async_init_done(dev, 0);
    case SX127X_STATE_CONFIG_RXC:
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
    case SX127X_STATE_CONFIG_RXC_PENDING_STOP:
      sx127x_rfp_end_rxc(pv, 0);
      break;
    default:
      abort();
  }

  if (!pv->bcrun && (pv->icount != (pv->bc_status & STATUS_IRQ_MSK)))
    sx127x_bytecode_start(pv, &sx127x_entry_irq, 0, 0);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_DEBUG)
static void sx127x_print_registers(struct sx127x_private_s *pv)
{
  for (uint32_t i = 0; i < 112; i++)
    printk(" 0x%02x: 0x%x\n", i+1, pv->dump[i]);
}
#endif 

/* ******************************** Device init/cleanup **********************/

static DEV_INIT(sx127x_init)
{
  struct sx127x_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->dev = dev;

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_DEBUG)
  pv->dump = mem_alloc(113, (mem_scope_sys));
  if (!pv->dump) 
    return -ENOMEM;
#endif

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
  srq->base.base.pvdata = dev;

  /* Init GPIO stuff */

  iomux_demux_t   loc[SX127X_PIN_COUNT];
  iomux_io_id_t   id[SX127X_PIN_COUNT];

  static const gpio_width_t pin_wmap[SX127X_PIN_COUNT] = {1, 1, 1, 1};

  if (device_iomux_setup(dev, ">rst <dio0 <dio2 <dio4", loc, id, NULL))
    goto err_timer;

  for (uint8_t i = 0; i < SX127X_PIN_COUNT; i++)
    pv->pin_map[i] = id[i];

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

  dev_request_queue_init(&pv->queue);

  kroutine_init_deferred(&srq->base.base.kr, &sx127x_spi_rq_done);
  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  /* Disable bytecode trace */
  bc_set_trace(&srq->vm, 0, 0);

  /* irq */
  device_irq_source_init(dev, pv->src_ep, 2, &sx127x_irq_process);

  if (device_irq_source_link(dev, pv->src_ep, 2, -1))
    goto err_timer;

  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  sx127x_bytecode_start(pv, &sx127x_entry_reset, 0); 

  pv->state = SX127X_STATE_INITIALISING;

  dev->drv_pv = pv;

  return -EAGAIN;

err_link:
  device_irq_source_unlink(dev, pv->src_ep, 2);
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
      assert(dev_request_queue_isempty(&pv->queue));
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
