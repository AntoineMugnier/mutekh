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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2022, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/pcm.h>
#include <device/class/iomux.h>
#ifdef CONFIG_DRIVER_NRF52_I2S_POWERGATE
#include <device/clock.h>
#endif

#include <arch/nrf5x/i2s.h>

#define I2S_ADDR NRF_PERIPHERAL_ADDR(NRF5X_I2S)

#define BUFFER_COUNT_L2 2
#define BUFFER_COUNT (1 << BUFFER_COUNT_L2)
#define BUFFER_IDX_MASK (BUFFER_COUNT - 1)
#define BUFFER_IDX_WRAP_MASK (BUFFER_COUNT | BUFFER_IDX_MASK)

struct nrf52_i2s_pv_s
{
  struct dev_pcm_rq_s *rq;
  struct dev_irq_src_s irq_ep;
  uint32_t current_session_id, next_session_id;

  bool_t mclk_master, has_in, has_out, should_start;
  uint32_t mclk_rate;
  uintptr_t paired_buffer_offset;

  void* txptr[BUFFER_COUNT], rxptr[BUFFER_COUNT];
  uint8_t txpw, rxpw, twpr, rxpr;

#ifdef CONFIG_DRIVER_NRF52_I2S_POWERGATE
  struct dev_clock_sink_ep_s power_source;
#endif
};

DRIVER_PV(struct nrf52_i2s_pv_s);

static DEV_IRQ_SRC_PROCESS(nrf52_i2s_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf52_i2s_pv_s *pv = dev->drv_pv;
  struct dev_pcm_rq_s *rq = pv->rq;

  logk_trace("%s\n", __FUNCTION__);

  if (nrf_event_check(I2S_ADDR, NRF_I2S_RXPTRUPD)) {
    nrf_event_clear(I2S_ADDR, NRF_I2S_RXPTRUPD);
    nrf_reg_set(I2S_ADDR, NRF_I2S_RXD_PTR, )
  }
}

static
error_t nrf5x_i2s_rq_init(
  struct nrf52_i2s_pv_s *pv,
  struct dev_pcm_rq_s *rq)
{
  enum used_channel_e {
    CHAN_I_L = 1,
    CHAN_I_R = 2,
    CHAN_O_L = 4,
    CHAN_O_R = 8,
    CHAN_O_LR = CHAN_O_R | CHAN_O_L,
    CHAN_I_LR = CHAN_I_R | CHAN_I_L,
    CHAN_IO_L = CHAN_I_L | CHAN_O_L,
    CHAN_IO_R = CHAN_I_R | CHAN_O_R,
    CHAN_IO_LR = CHAN_IO_R | CHAN_IO_L,
  };
  uint8_t used_channels, sample_size, stride;
  struct dev_pcm_stream_s *chanmap[4];
  size_t stream;
  uint8_t *sample;
  uint32_t mode;

  if (pv->current_session_id)
    return -EBUSY;

  if (rq->stream_count == 0)
    return -EINVAL;

  if (rq->stream_count > 4)
    return -ENOTSUP;

  used_channels = 0;
  for (stream = 0; stream < rq->stream_count; ++stream) {
    struct dev_pcm_stream_s *s = &rq->stream[stream];
    uint8_t c = 0;

    if (s->channel_id > 1)
      return -EINVAL;

    if (s->sample_type != rq->stream[0].sample_type)
      return -EINVAL;
    
    if (s->direction == DEV_PCM_DIR_INPUT) {
      if (!pv->has_in)
        return -ENOTSUP;

      c = s->channel_id;
    } else {
      if (!pv->has_out)
        return -ENOTSUP;

      c = 2 + s->channel_id;
    }

    if (used_channels & bit(c))
      return -EINVAL;

    if (s->buffer[0] || s->buffer[1])
      return -EINVAL;

    used_channels |= bit(c);
    chanmap[c] = s;
  }

  switch (used_channels & 0xf) {
  case CHAN_I_L:
  case CHAN_I_R:
  case CHAN_O_L:
  case CHAN_O_R:
  case CHAN_I_LR:
  case CHAN_O_LR:
  case CHAN_IO_LR:
  case CHAN_IO_L:
  case CHAN_IO_R:
    break;
  default:
    return -ENOTSUP;
  }

  // Sanity checks done, there should be no failure after this switch.
  
  switch (rq->stream[0].sample_type) {
  case DEV_PCM_DT_INT16LE:
  case DEV_PCM_DT_SINT16LE:
    nrf_reg_set(I2S_ADDR, NRF_I2S_SWIDTH, NRF_I2S_SWIDTH_16);
    sample_size = 2;
    break;
  case DEV_PCM_DT_INT8:
  case DEV_PCM_DT_SINT8:
    nrf_reg_set(I2S_ADDR, NRF_I2S_SWIDTH, NRF_I2S_SWIDTH_8);
    sample_size = 1;
    break;
  case DEV_PCM_DT_INT24LE:
  case DEV_PCM_DT_SINT24LE:
    nrf_reg_set(I2S_ADDR, NRF_I2S_SWIDTH, NRF_I2S_SWIDTH_24);
    sample_size = 4;
    break;
  default:
    return -ENOTSUP;
  }

  nrf_reg_set(I2S_ADDR, NRF_I2S_TXEN, !!(used_channels & CHAN_O_LR));
  nrf_reg_set(I2S_ADDR, NRF_I2S_RXEN, !!(used_channels & CHAN_I_LR));
  if ((used_channels & CHAN_IO_L) && (used_channels & CHAN_IO_R)) {
    stride = 2;
    nrf_reg_set(I2S_ADDR, NRF_I2S_CHANNELS, NRF_I2S_CHANNEL_LR);
  } else if (used_channels & CHAN_IO_L) {
    stride = 1;
    nrf_reg_set(I2S_ADDR, NRF_I2S_CHANNELS, NRF_I2S_CHANNEL_L);
  } else {
    stride = 1;
    nrf_reg_set(I2S_ADDR, NRF_I2S_CHANNELS, NRF_I2S_CHANNEL_R);
  }

  size_t total = rq->sample_count
    * sample_size
    * BUFFER_COUNT
    * rq->stream_count;

  sample = mem_alloc(total, mem_scope_sys);
  if (!sample)
    return -ENOMEM;
  // mem layout:
  // IN0  LRLRLRLR... [rq->sample_count * sample_size
  // OUT0 LRLRLRLR...   * rq->stream_count]
  // IN1  LRLRLRLR...  > * BUFFER_COUNT
  // OUT1 LRLRLRLR...  >
  // ...

  memset(sample, 0, total);
  
  pv->paired_buffer_offset
    = !!(used_channels & CHAN_IO_L) != !!(used_channels & CHAN_IO_R) ? sample_size : 0;
  uintptr_t out_buffer_offset = (used_channels & CHAN_I_LR) ? (sample_size * rq->sample_count) : 0;
  uintptr_t next_buffer_offset = sample_size * rq->stream_count * rq->sample_count;

  if (used_channels & CHAN_I_LR) {
    // Has input
    for (uint8_t b = 0; i < BUFFER_COUNT; ++i)
      pv->rxptr[b] = (void*)(sample + next_buffer_offset * b);
  } else {
    for (uint8_t b = 0; i < BUFFER_COUNT; ++i)
      pv->rxptr[b] = NULL;
  }

  if (used_channels & CHAN_O_LR) {
    // Has output
    for (uint8_t b = 0; i < BUFFER_COUNT; ++i)
      pv->txptr[b] = (void*)(sample + out_buffer_offset + next_buffer_offset * b);
  } else {
    for (uint8_t b = 0; i < BUFFER_COUNT; ++i)
      pv->txptr[b] = NULL;
  }
  
  for (uint8_t c = 0; c < 4; ++c) {
    struct dev_pcm_stream_s *s = chanmap[c];
    uintptr_t base;

    if (!s)
      continue;

    s->stride = sample_size * stride;

    if (bit(c) & CHAN_O_LR)
      base = pv->txptr[0];
    else
      base = pv->rxptr[0];

    if (bit(c) & CHAN_IO_R)
      // If we have R only, paired_buffer_offset is 0. Dont bother checking this.
      base += pv->paired_buffer_offset;

    s->buffer = (void*)base;
  }

  pv->txpw = 0;
  pv->txpr = 0;
  pv->rxpw = BUFFER_COUNT;
  pv->rxpr = 0;
  pv->should_start = 1;

  rq->error = 0;
  pv->current_session_id = ++(pv->next_session_id);
  rq->session_id = pv->current_session_id;

  dev_pcm_rq_done(rq);

  return 0;
}

static
error_t nrf5x_i2s_rq_stream(
  struct nrf52_i2s_pv_s *pv,
  struct dev_pcm_rq_s *rq)
{
  if (pv->current_session_id != rq->session_id)
    return -ENOENT;

  if (pv->txptr[0] && (pv->txpw - pv->txpr) & BUFFER_IDX_WRAP_MASK >= BUFFER_COUNT)
    goto blocked;

  if (pv->rxptr[0] && (pv->rxpw - pv->rxpr) & BUFFER_IDX_WRAP_MASK >= BUFFER_COUNT)
    goto blocked;

  rq->

  if (pv->rxptr)
    nrf_reg_set(I2S_ADDR, NRF_I2S_RXD_PTR, pv->rxptr[rq->offline_buffer_index]);

  rq->offline_buffer_index ^= 1;

  if (rq->action_pending & ACTION_START)
    nrf_task_trigger(I2S_ADDR, NRF_I2S_START);

  rq->action_pending = 0;

  pv->rq = NULL;
  dev_pcm_rq_done(rq);


  nrf_reg_set(I2S_ADDR, NRF_I2S_ENABLE, 1);

#ifdef CONFIG_DRIVER_NRF52_I2S_POWERGATE
  dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_POWER);
#endif


}

static
error_t nrf5x_i2s_rq_close(
  struct nrf52_i2s_pv_s *pv,
  struct dev_pcm_rq_s *rq)
{
}

static DEV_PCM_REQUEST(nrf52_i2s_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf52_i2s_pv_s *pv = dev->drv_pv;

  logk_debug("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->rq)
    return -EBUSY;

  switch (rq->op) {
  case DEV_PCM_OP_INIT:
    return nrf5x_i2s_rq_init(pv, rq);

  case DEV_PCM_OP_STREAM:
    return nrf5x_i2s_rq_stream(pv, rq);

  case DEV_PCM_OP_CLOSE:
    return nrf5x_i2s_rq_close(pv, rq);

  default:
    return -EINVAL;
  }
}

#define nrf52_i2s_use dev_use_generic

static DEV_INIT(nrf52_i2s_init)
{
  error_t err = 0;
  struct nrf52_i2s_pv_s *pv;
  iomux_io_id_t id[5];
  uintptr_t mclk_rate = 0, mclk_ratio = 0;
  uint32_t mode = NRF_I2S_MODE_MASTER;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         I2S_ADDR == addr);

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  err = device_iomux_setup(dev, ">wclk >bclk >mclk? >dout? <din?", NULL, id, NULL);
  if (err) {
    mode = NRF_I2S_MODE_SLAVE;

    err = device_iomux_setup(dev, "<wclk <bclk >mclk? >dout? <din?", NULL, id, NULL);
    if (err)
      goto free_pv;
  }

  pv->mclk_master = pv->io[2] != IOMUX_INVALID_ID;
  pv->has_out = pv->io[3] != IOMUX_INVALID_ID;
  pv->has_in = pv->io[4] != IOMUX_INVALID_ID;

  if (pv->mclk_master) {
    err = device_get_param_uint(dev, "mclk_rate", &mclk_rate);
    if (err)
      goto free_pv;

    err = device_get_param_uint(dev, "mclk_ratio", &mclk_ratio);
    if (err)
      mclk_ratio = NRF_I2S_RATIO_256X;
  }

#ifdef CONFIG_DRIVER_NRF52_I2S_POWERGATE
  err = dev_drv_clock_init(dev, &pv->power_source, 0, 0, NULL);
  if (err)
    goto free_pv;
#endif

  nrf_it_disable_mask(I2S_ADDR, -1);

  nrf_event_clear(I2S_ADDR, NRF_I2S_STARTED);
  nrf_event_clear(I2S_ADDR, NRF_I2S_STOPPED);
  nrf_event_clear(I2S_ADDR, NRF_I2S_END);

  nrf_reg_set(I2S_ADDR, NRF_I2S_ENABLE, 0);
  nrf_reg_set(I2S_ADDR, NRF_I2S_MODE, mode);
  //nrf_reg_set(I2S_ADDR, NRF_I2S_TXEN, );
  //nrf_reg_set(I2S_ADDR, NRF_I2S_RXEN, );
  nrf_reg_set(I2S_ADDR, NRF_I2S_MCKEN, !!mclk_rate);
  nrf_reg_set(I2S_ADDR, NRF_I2S_MCKFREQ, mclk_rate);
  nrf_reg_set(I2S_ADDR, NRF_I2S_RATIO, mclk_ratio);
  //nrf_reg_set(I2S_ADDR, NRF_I2S_SWIDTH, );
  nrf_reg_set(I2S_ADDR, NRF_I2S_ALIGN, NRF_I2S_ALIGN_LEFT);
  nrf_reg_set(I2S_ADDR, NRF_I2S_FORMAT, NRF_I2S_FORMAT_I2S);
  //nrf_reg_set(I2S_ADDR, NRF_I2S_CHANNELS, );
  //nrf_reg_set(I2S_ADDR, NRF_I2S_RXD_PTR, );
  //nrf_reg_set(I2S_ADDR, NRF_I2S_TXD_PTR, );
  //nrf_reg_set(I2S_ADDR, NRF_I2S_RTXD_MAXCNT, );
  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_LRCK, id[0]);
  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_SCK, id[1]);
  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_MCK, pv->mclk_master ? id[2] : -1);
  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_DOUT, pv->has_out ? id[3] : -1);
  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_DIN, pv->has_in ? id[4] : -1);
  
  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf52_i2s_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_pv;

  kroutine_init_idle(&pv->deleter, nrf52_i2s_ended);

  return 0;
  
 free_pv:
  mem_free(pv);

  return -1;
}

static DEV_CLEANUP(nrf52_i2s_cleanup)
{
  struct nrf52_i2s_pv_s *pv = dev->drv_pv;

  if (pv->rq)
    return -EBUSY;

  nrf_it_disable_mask(I2S_ADDR, -1);

#ifdef CONFIG_DRIVER_NRF52_I2S_POWERGATE
  dev_drv_clock_cleanup(dev, &pv->power_source);
#endif

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  nrf_task_trigger(I2S_ADDR, NRF_I2S_STOPPED);

  nrf_reg_set(I2S_ADDR, NRF_I2S_ENABLE, 0);

  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_CLK, (uint32_t)-1);
  nrf_reg_set(I2S_ADDR, NRF_I2S_PSEL_DIN, (uint32_t)-1);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf52_i2s_drv, 0, "nRF52 I2S", nrf52_i2s,
               DRIVER_PCM_METHODS(nrf52_i2s));

DRIVER_REGISTER(nrf52_i2s_drv);
