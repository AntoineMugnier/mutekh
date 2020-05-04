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
    Copyright (c) 2016 Julien Peeters <contact@julienpeeters.net>

*/

#include "sx127x_lora_spi.h"

#include <hexo/endian.h>

#define CONFIG_DRIVER_RFPACKET_CLK_SOURCE

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_DEBUG)
# include <mutek/printk.h>
# define dprintk(...) printk("[sx127x] " __VA_ARGS__)
#else
# define dprintk(...) do {} while(0)
#endif

/* ******************************** RF Packet ********************************/

struct sx127x_config_bw_s
{
  uint32_t BITFIELD(bw,24);
  uint8_t  bits;
};

static const struct sx127x_config_bw_s sx127x_config_bw_tbl[] =
{
  {   7800, 0x0 },
  {  10400, 0x1 },
  {  15600, 0x2 },
  {  20800, 0x3 },
  {  31250, 0x4 },
  {  41700, 0x5 },
  {  62500, 0x6 },
  { 125000, 0x7 },
  { 250000, 0x8 },
  { 500000, 0x9 },
};

static inline
uint32_t sx127x_config_id_get(struct dev_rfpacket_rq_s const * rq)
{
  return rq->pk_cfg->cache.id                     |
         (((uint32_t)rq->rf_cfg->cache.id) <<  8) |
         (((uint32_t)rq->channel)          << 16);
}

static inline
void sx127x_config_pk_update(struct dev_rfpacket_pk_cfg_lora_s const * cfg,
                             uint8_t *                                 modemcfg1,
                             uint8_t *                                 modemcfg2)
{
  *modemcfg1 &= 0xf0;
  *modemcfg2 &= 0xfb;

  if (!cfg->header)
    *modemcfg1 |= 0x1;

  *modemcfg1 |= (cfg->crate & 0x7) << 1;

  if (cfg->crc)
    *modemcfg2 |= 0x4;
}

static inline
void sx127x_config_rf_update(struct dev_rfpacket_rf_cfg_lora_s const * cfg,
                             uint8_t *                                 modemcfg1,
                             uint8_t *                                 modemcfg2,
                             uint8_t *                                 modemcfg3)
{
  uint8_t bw = 0;
  uint_fast8_t i;

  for (i = 0; i < ARRAY_SIZE(sx127x_config_bw_tbl); ++i)
    {
      if (sx127x_config_bw_tbl[i].bw == cfg->common.rx_bw)
        bw = sx127x_config_bw_tbl[i].bits;
    }

  *modemcfg1 &= 0x0f;
  *modemcfg2 &= 0x07; /* force TX continuous to zero */
  *modemcfg3  = 0;

  *modemcfg1 |= (bw & 0xf) << 4;
  *modemcfg2 |= (cfg->spreading & 0xf) << 4;

  /* If the symbol duration is more than 16ms, than low data rate optimization
   * must be enabled. */
  uint8_t const ts = (uint16_t)(1 << cfg->spreading) * 1000 / cfg->common.rx_bw;

  if (ts >= 16)
    *modemcfg3 |= 0x8;
}

static inline
void sx127x_config_freq_update(struct dev_rfpacket_rf_cfg_lora_s const * rf_cfg,
                               int16_t                                   channel,
                               uint8_t                                   data[4])
{
  int32_t  const freq_offset = channel * rf_cfg->common.chan_spacing;
  uint64_t const freq = (int64_t)rf_cfg->common.frequency + freq_offset;

  /* Truncate to lowest 32 bits */
  uint32_t frf = (freq << 19) / CONFIG_DRIVER_RFPACKET_SX127X_FREQ_XO;
  endian_be32_na_store(data, frf << 8);
}

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0

#define SX127X_CONFIG_CACHE_ID(key) \
  (key % CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE)

static inline
void sx127x_config_pk_cache_update(struct sx127x_private_s *                 pv,
                                   struct dev_rfpacket_pk_cfg_lora_s const * cfg,
                                   struct sx127x_pk_cfg_cache_entry_s *      entry)
{
  sx127x_config_pk_update(cfg, &entry->modemcfg1_2[0], &entry->modemcfg1_2[1]);

  /* Required for static const config that are not marked as dirty and
     cannot be updated with store instruction. */
  if (cfg->base.cache.dirty)
    {
      ((struct dev_rfpacket_pk_cfg_lora_s *)cfg)->base.cache.dirty = 0;

      dprintk("CACHE: pk cache entry validated id:%u cfg:%p (entry:%p)\n",
              cfg->base.cache.id, cfg, entry);
    }

  entry->cfg = cfg;
  assert(entry->cfg->base.cache.dirty == 0);
}

static inline
void sx127x_config_rf_cache_update(struct sx127x_private_s *                 pv,
                                   struct dev_rfpacket_rf_cfg_lora_s const * cfg,
                                   struct sx127x_rf_cfg_cache_entry_s *      entry)
{
  sx127x_config_rf_update(cfg, &entry->modemcfg1_2[0], &entry->modemcfg1_2[1],
                          &entry->modemcfg3);

  /* Required for static const config that are not marked as dirty and
     cannot be updated with store instruction. */
  if (cfg->base.cache.dirty)
    {
      ((struct dev_rfpacket_rf_cfg_lora_s *)cfg)->base.cache.dirty = 0;

      dprintk("CACHE: rf cache entry validated id:%u cfg:%p (entry:%p)\n",
              cfg->base.cache.id, cfg, entry);
    }

  entry->cfg = cfg;
  assert(entry->cfg->base.cache.dirty == 0);
}

static inline
void sx127x_config_freq_cache_update(struct sx127x_private_s *                 pv,
                                     struct dev_rfpacket_rf_cfg_lora_s const * rf_cfg,
                                     int16_t                                   channel,
                                     struct sx127x_freq_cache_entry_s *        entry)
{
  sx127x_config_freq_update(rf_cfg, channel, &entry->data[0]);
  dprintk("CACHE: fq cache entry validated id:%u!\n", channel);
}

static
bool_t sx127x_config_cache_hit(struct sx127x_private_s *        pv,
                               struct dev_rfpacket_rq_s const * rq,
                               uint_fast8_t *                   pk_cache_idx,
                               uint_fast8_t *                   rf_cache_idx,
                               uint_fast8_t *                   fq_cache_idx)
{
  struct dev_rfpacket_pk_cfg_lora_s const * pk_cfg =
    const_dev_rfpacket_pk_cfg_lora_s_cast(rq->pk_cfg);

  struct dev_rfpacket_rf_cfg_lora_s const * rf_cfg =
    const_dev_rfpacket_rf_cfg_lora_s_cast(rq->rf_cfg);

  *pk_cache_idx = SX127X_CONFIG_CACHE_ID(pk_cfg->base.cache.id);
  *rf_cache_idx = SX127X_CONFIG_CACHE_ID(rf_cfg->base.cache.id);
  *fq_cache_idx = SX127X_CONFIG_CACHE_ID(rq->channel);

  pv->dirty = 0;

  dprintk("CACHE: pk cfg cacheid:%u cachecfg:%p dirty:%u cfg:%p\n", *pk_cache_idx,
          pv->pk_cfg_cache[*pk_cache_idx].cfg, pk_cfg->base.cache.dirty, pk_cfg);
  if (pv->pk_cfg_cache[*pk_cache_idx].cfg != pk_cfg || pk_cfg->base.cache.dirty)
    pv->dirty |= SX127X_CFG_CACHE_MSK_PK;

  dprintk("CACHE: rf cfg cacheid:%u cachecfg:%p dirty:%u cfg:%p\n", *rf_cache_idx,
          pv->rf_cfg_cache[*rf_cache_idx].cfg, rf_cfg->base.cache.dirty, rf_cfg);
  if (pv->rf_cfg_cache[*rf_cache_idx].cfg != rf_cfg || rf_cfg->base.cache.dirty)
    pv->dirty |= SX127X_CFG_CACHE_MSK_RF;

  dprintk("CACHE: fq cfg cacheid:%u cache:%d channel:%d\n", *fq_cache_idx,
          pv->freq_cache[*fq_cache_idx].channel, rq->channel);
  if ((pv->dirty & SX127X_CFG_CACHE_MSK_RF) ||
      pv->freq_cache[*fq_cache_idx].channel != rq->channel)
    pv->dirty |= SX127X_CFG_CACHE_MSK_FQ;

  return pv->dirty == 0;
}

static
void sx127x_config_cache_update(struct sx127x_private_s *        pv,
                                struct dev_rfpacket_rq_s const * rq,
                                uint_fast8_t                     pk_cache_idx,
                                uint_fast8_t                     rf_cache_idx,
                                uint_fast8_t                     fq_cache_idx)
{
  if (pv->dirty & SX127X_CFG_CACHE_MSK_PK)
    sx127x_config_pk_cache_update(pv, dev_rfpacket_pk_cfg_lora_s_cast(rq->pk_cfg),
                                  &pv->pk_cfg_cache[pk_cache_idx]);

  if (pv->dirty & SX127X_CFG_CACHE_MSK_RF)
    sx127x_config_rf_cache_update(pv, dev_rfpacket_rf_cfg_lora_s_cast(rq->rf_cfg),
                                  &pv->rf_cfg_cache[rf_cache_idx]);

  if (pv->dirty & SX127X_CFG_CACHE_MSK_FQ)
    sx127x_config_freq_cache_update(pv, dev_rfpacket_rf_cfg_lora_s_cast(rq->rf_cfg),
                                    rq->channel, &pv->freq_cache[fq_cache_idx]);
}

#endif // CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0

static void sx127x_config_update(struct device_s * dev);
static void sx127x_process_request(struct device_s * dev);

static
void sx127x_idle(struct device_s * dev)
{
  struct sx127x_private_s * pv = dev->drv_pv;

  pv->state = SX127X_STATE_IDLE;

  // TODO: sleep/low-power
  // if (sleep)
  //   pv->state = SX127X_STATE_SLEEP;
}

static
void sx127x_next_request(struct device_s * dev)
{
  struct sx127x_private_s  * pv   = dev->drv_pv;
  struct dev_rfpacket_rq_s * rq   = dev_rfpacket_rq_head(&pv->queue);

  bool_t changed = 0;

  if (!rq && pv->rx_cont_rq)
    {
      dprintk("no pending request, then operate RX continuous.\n");
      rq = pv->rx_cont_rq;
    }

  if (!rq)
    {
      sx127x_idle(dev);
      return;
    }

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
  uint_fast8_t pki, rfi, fqi;
  if (!sx127x_config_cache_hit(pv, rq, &pki, &rfi, &fqi))
    {
      sx127x_config_cache_update(pv, rq, pki, rfi, fqi);
      changed = 1;
    }
  else
    {
#endif
      uint32_t const rq_cfg_id = sx127x_config_id_get(rq);
      if (rq_cfg_id != pv->last_cfg_id)
        changed = 1;
#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
    }
#endif

  dprintk("last_cfg:0x%x new_cfg:0x%x changed:%u\n", pv->last_cfg_id,
          rq_cfg_id, changed);

  /* Save current request */
  pv->next_rq = rq;

  if (changed)
    sx127x_config_update(dev);
  else
    sx127x_process_request(dev);
}

static
void sx127x_rx_start(struct device_s * dev, struct dev_rfpacket_rq_s * rq)
{
  struct sx127x_private_s * pv = dev->drv_pv;

  pv->next_rq = rq;

  switch (rq->type)
    {
    default:
      break;

    case DEV_RFPACKET_RQ_RX:
      dprintk("RX: waiting for a single packet...\n");

      if (!dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_rx,
                                  (1 << R_ARG0) | (1 << R_ARG1),
                                  (uintptr_t)&rq->deadline, rq->lifetime))
        pv->state = SX127X_STATE_RX;
      break;

    case DEV_RFPACKET_RQ_RX_CONT:
      dprintk("RX: waiting for packets...\n");

      if (!dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_rx_cont, 0))
        pv->state = SX127X_STATE_RX_CONTINUOUS;
      break;
    }
}

static void sx127x_rx_end(struct device_s * dev);
static void sx127x_rx_packet_end(struct device_s * dev);

static
void sx127x_rx_end(struct device_s * dev)
{
  struct sx127x_private_s * pv = dev->drv_pv;

  pv->done &= ~SX127X_RX_MASK;

  dprintk("packet received!\n");

  pv->rx_last = pv->next_rq->rx_alloc(pv->next_rq, CONFIG_DRIVER_RFPACKET_SX127X_PACKET_MTU);
  if (!pv->rx_last)
    {
      dprintk("RX: no more space for new buffers.\n");
      /* Restart current RX request */
      sx127x_next_request(dev);
      return;
    }

#if defined(CONFIG_DEVICE_SPI_BYTECODE_TIMER)
  pv->rx_last->timestamp = pv->timestamp;
#endif

  dprintk("RX: reading packet from fifo (rx:%p)...\n", pv->rx_last);
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_rx_packet, 0);
}

static
void sx127x_rx_packet_end(struct device_s * dev)
{
  struct sx127x_private_s *  pv = dev->drv_pv;
  struct dev_rfpacket_rx_s * rx = pv->rx_last;

  pv->done &= ~SX127X_RX_PACKET_MASK;

  /* Compute RSSI in 0.125 dbm step */
  rx->snr = (int16_t)((pv->bc_pkt_infos >> 8) & 0xff) * 8 / 4;
  if (rx->snr >= 0)
    rx->rssi = ((int16_t)(pv->bc_pkt_infos & 0xff) * 16 * 8 / 15 - 157 * 8);
  else
    rx->rssi = ((int16_t)(pv->bc_pkt_infos & 0xff) - 157) * 8;

  dprintk("RX: packet size:%u %P\n", rx->size, rx->buf, rx->size);
  assert(rx->size <= CONFIG_DRIVER_RFPACKET_SX127X_PACKET_MTU);

  if (pv->next_rq->type != DEV_RFPACKET_RQ_RX_CONT)
    {
      dev_rfpacket_rq_pop(&pv->queue);
      pv->next_rq = NULL;
    }
  pv->rx_last = NULL;

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_STATS)
  ++pv->stats.rx_count;
#endif

  lock_release(&dev->lock);
  kroutine_exec(&rx->kr);
  lock_spin(&dev->lock);

  dprintk("RX: packet fetched and kroutine called!\n");

  /* Process next request if any */
  sx127x_next_request(dev);
}

static
void sx127x_rx_timeout(struct device_s * dev)
{
  struct sx127x_private_s * pv = dev->drv_pv;

  pv->done &= ~SX127X_RX_TIMEOUT_MASK;

  if (pv->next_rq && pv->next_rq != pv->rx_cont_rq)
    {
      struct dev_rfpacket_rq_s * rq = pv->next_rq;
      dev_rfpacket_rq_pop(&pv->queue);

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_STATS)
  ++pv->stats.rx_err_count;
#endif

      rq->error = -ETIMEDOUT;
      lock_release(&dev->lock);
      dev_rfpacket_rq_done(rq);
      lock_spin(&dev->lock);

      pv->next_rq = NULL;
    }

  /* Process next request if any */
  sx127x_next_request(dev);
}

static
void sx127x_tx_start(struct device_s * dev, struct dev_rfpacket_rq_s * rq)
{
  struct sx127x_private_s * pv = dev->drv_pv;

  pv->next_rq = rq;

  dprintk("TX: packet @%p %P\n", pv->next_rq->tx_buf, pv->next_rq->tx_buf, pv->next_rq->tx_size);

  if (!dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_tx,
                              (1 << R_ARG0), (uintptr_t)&rq->deadline))
    pv->state = SX127X_STATE_TX;
}

static
void sx127x_tx_end(struct device_s * dev)
{
  struct sx127x_private_s *  pv = dev->drv_pv;
  struct dev_rfpacket_rq_s * rq = pv->next_rq;

  pv->done &= ~SX127X_TX_MASK;

  dprintk("TX: sent %d bytes!\n", rq->tx_size);

  dev_rfpacket_rq_pop(&pv->queue);
  pv->next_rq = NULL;

#if defined(CONFIG_DEVICE_SPI_BYTECODE_TIMER)
  rq->tx_timestamp = pv->timestamp;
#endif

#if defined(CONFIG_DRIVER_RFPACKET_SX127X_STATS)
  ++pv->stats.tx_count;
#endif

  lock_release(&dev->lock);
  dev_rfpacket_rq_done(rq);
  lock_spin(&dev->lock);

  /* Process next request if any */
  sx127x_next_request(dev);
}

static
void sx127x_config_update(struct device_s * dev)
{
  struct sx127x_private_s * pv = dev->drv_pv;

  struct dev_rfpacket_rq_s * rq =
    dev_rfpacket_rq_head(&pv->queue);

  if (!rq)
    rq = pv->rx_cont_rq;

  assert(rq && "Cannot update configuration without request");

  const struct dev_rfpacket_pk_cfg_lora_s * pk_cfg =
    const_dev_rfpacket_pk_cfg_lora_s_cast(rq->pk_cfg);

  const struct dev_rfpacket_rf_cfg_lora_s * rf_cfg =
    const_dev_rfpacket_rf_cfg_lora_s_cast(rq->rf_cfg);

  /* Retrieve the last sent config. */
  uint64_t const last_cfg  = endian_le64_na_load(pv->bkp_config.data);
  uint32_t const last_freq = endian_be32_na_load(pv->bkp_freq);

  /* Start updating the configuration. */
  pv->bkp_config.sw = pk_cfg->sw_value;
  pv->bkp_config.pl = pk_cfg->pb_len;

  switch (rq->type)
    {
    default:
      assert(!"unsupported request type");
      break;

    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
      pv->bkp_config.iq = rf_cfg->iq_inverted ? 0x67 : 0x27;
      break;

    case DEV_RFPACKET_RQ_TX:
      pv->bkp_config.iq = rf_cfg->iq_inverted ? 0x26 : 0x27;
      break;
    }

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
  uint_fast8_t const pki = SX127X_CONFIG_CACHE_ID(pk_cfg->base.cache.id);
  uint_fast8_t const rfi = SX127X_CONFIG_CACHE_ID(rf_cfg->base.cache.id);
  uint_fast8_t const fqi = SX127X_CONFIG_CACHE_ID((uint32_t)rq->channel);

  dprintk("pki:%u rfi:%u fqi:%u\n", pki, rfi, fqi);

  pv->bkp_config.modemcfg[0] = pv->pk_cfg_cache[pki].modemcfg[0]
                             | pv->rf_cfg_cache[rfi].modemcfg[0];

  pv->bkp_config.modemcfg[1] = pv->pk_cfg_cache[pki].modemcfg[1]
                             | pv->rf_cfg_cache[rfi].modemcfg[1];

  pv->bkp_config.modemcfg[2] = pv->rf_cfg_cache[rfi].modemcfg[2];

  memcpy(&pv->freq[0], &pv->freq_cache[fqi].data[0], 4);
#else
  sx127x_config_pk_update(pk_cfg, &pv->bkp_config.modemcfg[0],
                          &pv->bkp_config.modemcfg[1]);
  sx127x_config_rf_update(rf_cfg, &pv->bkp_config.modemcfg[0],
                          &pv->bkp_config.modemcfg[1], &pv->bkp_config.modemcfg[2]);
  sx127x_config_freq_update(rf_cfg, rq->channel, &pv->bkp_freq[0]);
#endif

  /* Retrieve the new config to be sent if needed. */
  uint64_t const new_cfg  = endian_le64_na_load(pv->bkp_config.data);
  uint32_t const new_freq = endian_be32_na_load(pv->bkp_freq);

  /* Compute the diff between the last and the new config. */
  uint8_t diff[8];
  endian_64_na_store(diff, last_cfg ^ new_cfg);

  pv->bkp_config_mask = 0;

  uint_fast8_t i;
  for (i = 0; i < 6; ++i)
    {
      if (diff[i] != 0)
        pv->bkp_config_mask |= 1 << i;
    }

  if (last_freq != new_freq)
    pv->bkp_config_mask |= 1 << 7;

  dprintk("last:%P new:%P mask:0x%02x\n", &last_cfg, 6, &new_cfg, 6,
          pv->bkp_config_mask);

  if (!dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_config,
                              (1 << R_ARG0), pv->bkp_config_mask))
    pv->state = SX127X_STATE_CONFIG;
}

static
void sx127x_config_end(struct device_s * dev)
{
  struct sx127x_private_s *  pv = dev->drv_pv;
  struct dev_rfpacket_rq_s * rq = pv->next_rq;

  assert(rq && "cannot acknowledge radio configuration without request");

  /* Clear done flag */
  pv->done &= ~SX127X_CFG_MASK;

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
  dprintk("transceiver config updated!\n");

  /* Set the cache as valid */
  pv->dirty = 0;
#endif

  /* Save last sent configuration */
  pv->last_cfg_id = sx127x_config_id_get(rq);

  /* Process next RF request */
  sx127x_process_request(dev);
}

static
void sx127x_process_request(struct device_s * dev)
{
  struct sx127x_private_s *  pv = dev->drv_pv;
  struct dev_rfpacket_rq_s * rq = pv->next_rq;

  assert(rq && "next request not defined");

  switch (rq->type)
    {
    case DEV_RFPACKET_RQ_TX:
      sx127x_tx_start(dev, rq);
      break;

    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
      sx127x_rx_start(dev, rq);
      break;

    default:
      rq->error = -ENOTSUP;
      dev_rfpacket_rq_pop(&pv->queue);

      lock_release(&dev->lock);
      dev_rfpacket_rq_done(rq);
      lock_spin(&dev->lock);

      sx127x_next_request(dev);
      break;
    }
}

static
DEV_RFPACKET_REQUEST(sx127x_request)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  bool_t  empty;
  va_list vl;

  LOCK_SPIN_IRQ(&dev->lock);

  empty = dev_rq_queue_isempty(&pv->queue);

  va_start(vl, accessor);

  while (1)
    {
      struct dev_rfpacket_rq_s * rq = va_arg(vl, struct dev_rfpacket_rq_s *);
      if (!rq)
        break;

      if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
        {
          if (pv->rx_cont_rq)
            {
              rq->error = -EBUSY;

              lock_release(&dev->lock);
              dev_rfpacket_rq_done(rq);
              lock_spin(&dev->lock);
            }
          else
            {
              pv->rx_cont_rq = rq;
            }
        }
      else
        {
          dev_rfpacket_rq_pushback(&pv->queue, rq);
        }
    }

  dprintk("new request empty:%u rxcont:%u\n", empty, pv->rx_cont_rq != NULL);
  if (empty || pv->rx_cont_rq)
    sx127x_next_request(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static
DEV_IRQ_SRC_PROCESS(sx127x_irq_process)
{
  struct device_s *         dev = ep->base.dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  lock_spin(&dev->lock);

#if defined(CONFIG_DEVICE_SPI_BYTECODE_TIMER)
  DEVICE_OP(pv->timer, get_value, &pv->timestamp, 0);
#endif

  dprintk("irq!\n");

  pv->icount++;
  if (dev_spi_bytecode_wakeup(&pv->spi, &pv->spi_rq))
    dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_irq, 0);

  lock_release(&dev->lock);
}

static
void sx127x_cancel_end(struct device_s * dev)
{
  struct sx127x_private_s *  pv = dev->drv_pv;

  pv->done &= ~SX127X_CANCEL_MASK;

  if (pv->rx_cont_rq == pv->next_rq)
    pv->rx_cont_rq = NULL;
  else
    dev_rfpacket_rq_pop(&pv->queue);
}

static
DEV_RFPACKET_CANCEL(sx127x_cancel)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  // If the current executing request is not a RX continuous, then cannot
  // cancel it.
  if (rq == pv->next_rq && pv->next_rq != pv->rx_cont_rq)
    err = -EBUSY;

  // Otherwise, cancel the request, being a RX continuous or a not already
  // scheduled request.
  else
    {
      if (pv->next_rq && pv->next_rq == pv->rx_cont_rq)
        {
          assert(rq == pv->rx_cont_rq && pv->rx_cont_rq == pv->next_rq);
          dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_cancel, 0);
          dprintk("start cancelling RX cont. request %p.\n", rq);
        }
      else
        {
          dev_rfpacket_rq_remove(&pv->queue, rq);
          dprintk("cancel request %p.\n", rq);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_RFPACKET_STATS(sx127x_stats)
{
#if defined(CONFIG_DRIVER_RFPACKET_SX127X_STATS)
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  *stats = pv->stats;
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

/* ******************************** RNG **************************************/

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)

static
DEV_CRYPTO_INFO(sx127x_crypto_info)
{
  memset(info, 0, sizeof(*info));

  info->name       = "Semtech SX127x Transceiver";
  info->modes_mask = 1 << DEV_CRYPTO_MODE_RANDOM;
  info->state_size = CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE;

  return 0;
}

static
void sx127x_crypto_rng_end(struct device_s * dev)
{
  struct sx127x_private_s * pv = dev->drv_pv;
  struct dev_crypto_rq_s *  rq = pv->crypto_rq;

  pv->done &= ~SX127X_RNG_MASK;

  if (rq->op & DEV_CRYPTO_FINALIZE)
    memcpy(rq->out, rq->ctx->state_data,
           __MIN(rq->len, CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE));

  rq->error = 0;

  lock_release(&dev->lock);
  dev_rfpacket_rq_done(rq);
  lock_spin(&dev->lock);

  pv->crypto_rq = NULL;

  /* Process next request, if any */
  sx127x_next_request(dev);
}

static
void sx127x_crypto_process_request(struct device_s * dev)
{
  struct sx127x_private_s * pv   = dev->drv_pv;
  struct dev_crypto_rq_s *  rq   = pv->crypto_rq;

  uint_fast8_t i;
  uint8_t *    state = rq->ctx->state_data;

  if (rq->op & DEV_CRYPTO_INIT)
    memset(rq->ctx->state_data, 0, CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE);

  if (rq->op & DEV_CRYPTO_INVERSE)
    {
      for (i = 0; i < __MIN(rq->ad_len, CONFIG_DRIVER_CRYPTO_SX127X_RNG_SIZE); ++i)
        state[i] |= rq->ad[i];
    }

  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sx127x_entry_rng, 0);
}

static
DEV_CRYPTO_REQUEST(sx127x_crypto_request)
{
  struct device_s *         dev = accessor->dev;
  struct sx127x_private_s * pv  = dev->drv_pv;

  rq->error = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->state != SX127X_STATE_IDLE)
    {
      rq->error = -EBUSY;
    }
  else
    {
      pv->crypto_rq = rq;
      sx127x_crypto_process_request(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (rq->error)
    dev_rfpacket_rq_done(rq);
}

#endif // CONFIG_DRIVER_CRYPTO_SX127X_RNG

/* ******************************** SPI **************************************/

static KROUTINE_EXEC(sx127x_spi_rq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s * srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *                   dev = srq->pvdata;
  struct sx127x_private_s *           pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (srq->error)
    abort();

  if (pv->icount != bc_get_reg(&srq->vm, R_ICOUNT))
    {
      dev_spi_bytecode_start(&pv->spi, srq, &sx127x_entry_irq, 0);
      goto end;
    }

#if defined(CONFIG_DRIVER_CRYPTO_SX127X_RNG)
  if (pv->crypto_rq != NULL && pv->done & SX127X_RNG_MASK)
    {
      pv->done &= ~SX127X_RNG_MASK;
      sx127x_crypto_rng_end(dev);
      goto end;
    }
#endif

  dprintk("SPI: done:0x%x\n", pv->done);
  dprintk("STATE: %d\n", pv->state);

  if (!pv->done)
    {
      if (pv->state == SX127X_STATE_RX_CONTINUOUS && !dev_rq_queue_isempty(&pv->queue))
        sx127x_next_request(dev);
      goto end;
    }

  if (pv->done & SX127X_INIT_MASK)
    {
      pv->done &= ~SX127X_INIT_MASK;
      /* Asynchronous init callback here */
      device_async_init_done(dev, 0);
    }

  if (pv->done & SX127X_CFG_MASK)
    sx127x_config_end(dev);

  if (pv->done & SX127X_RX_MASK)
    sx127x_rx_end(dev);

  if (pv->done & SX127X_RX_TIMEOUT_MASK)
    sx127x_rx_timeout(dev);

  if (pv->done & SX127X_RX_PACKET_MASK)
    sx127x_rx_packet_end(dev);

  if (pv->done & SX127X_TX_MASK)
    sx127x_tx_end(dev);

  if (pv->done & SX127X_CANCEL_MASK)
    sx127x_cancel_end(dev);

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

  struct dev_spi_ctrl_bytecode_rq_s * srq  = &pv->spi_rq;
  struct device_gpio_s *              gpio = NULL;

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
  /* Base 5 ms time */
  dev_timer_init_sec(pv->timer, &pv->delay_5ms, 0, 5, 1000);

  /* Start timer */
  if (device_start(&pv->timer->base))
    goto err_srq;
#endif

  srq->base.cs_cfg.polarity = DEV_SPI_ACTIVE_LOW;

  srq->pvdata = dev;

  /* Init GPIO stuff */

  static const gpio_width_t pin_wmap[3] = {1, 1, 1};

  // TODO check if correct syntax
  if (device_gpio_setup(gpio, dev, ">rst:1 <dio0:1 <dio3:1", pv->pin_map, NULL))
    goto err_timer;

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

  dev_rq_queue_init(&pv->queue);
#if defined(CONFIG_DRIVER_RFPACKET_SX127X_CRYPTO_RNG)
  dev_rq_queue_init(&pv->crypt_queue);
#endif

  dev_spi_ctrl_rq_init(&srq->base, &sx127x_spi_rq_done);
  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  /* Disable bytecode trace */
  bc_set_trace(&srq->vm, 0);

  /* irq io pin */
  device_irq_source_init(dev, pv->src_ep, 2, &sx127x_irq_process);

  if (device_irq_source_link(dev, pv->src_ep, 2, -1))
    goto err_timer;

  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  if (dev_spi_bytecode_start(&pv->spi, srq, &sx127x_entry_reset, 0))
    goto err_link;

#if CONFIG_DRIVER_RFPACKET_SX127X_CACHE_SIZE > 0
  pv->dirty = SX127X_CFG_CACHE_MSK_ALL;
#endif

  /* Set last config identifier to imposible value to force push the first
   * new config. */
  pv->last_cfg_id = -1;
  endian_64_na_store(pv->bkp_config.data, -1);
  endian_be32_na_store(pv->bkp_freq, -1);

  pv->state = SX127X_STATE_IDLE;

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
  struct sx127x_private_s * pv = dev->drv_pv;

  device_irq_source_unlink(dev, pv->src_ep, 1);

  dev_rq_queue_destroy(&pv->queue);
#if defined(CONFIG_DRIVER_RFPACKET_SX127X_CRYPTO_RNG)
  dev_rq_queue_destroy(&pv->crypt_queue);
#endif

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);

  mem_free(pv);
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
