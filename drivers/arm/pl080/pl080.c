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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#define LOGK_MODULE_ID "pl80"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/dma.h>

#include "pl080.h"
#include "drivers/arm/pl080.h"
#include "drivers/arm/identification.h"

struct pl080_desc_s
{
  uint32_t srcaddr;
  uint32_t destaddr;
  uint32_t lli;
  uint32_t control;
};

struct pl080_channel_s
{
  struct dev_dma_rq_s *running;
  struct pl080_desc_s *chain;
  uint32_t config;
  uint32_t desc_index;
};

struct pl080_pv_s
{
  uintptr_t base;

  struct dev_irq_src_s irq_ep[ARM_PL080_IRQ_COUNT];

  struct pl080_channel_s channel[0];
};

DRIVER_PV(struct pl080_pv_s);

static
uint8_t pl080_channel_count(struct pl080_pv_s *pv)
{
  uint32_t pid3 = cpu_mem_read_32(pv->base + 0xfec);
  return 2 << (pid3 & 0x7);
}

static
void pl080_channel_halt(struct pl080_pv_s *pv, uint8_t chan)
{
  uint32_t config = pl080_chan_get32(pv->base, chan, CONFIGURATION);
  config = pl080_ins_val(config, CONFIGURATION, H, 1);
  pl080_chan_set32(pv->base, chan, CONFIGURATION, config);

  for (;;) {
    config = pl080_chan_get32(pv->base, chan, CONFIGURATION);
    if (!pl080_get(config, CONFIGURATION, A))
      break;
  }

  config = pl080_ins_val(config, CONFIGURATION, E, 0);
  pl080_chan_set32(pv->base, chan, CONFIGURATION, config);

  config = pl080_ins_val(config, CONFIGURATION, H, 0);
  pl080_chan_set32(pv->base, chan, CONFIGURATION, config);
}

static
void pl080_descriptor_serialize(struct pl080_desc_s *phys,
                                enum dev_dma_rq_type_e type,
                                const struct dev_dma_desc_s *api,
                                struct pl080_desc_s *next)
{
  phys->lli = (uint32_t)next;

  uint32_t control = 0;
  control = pl080_ins_val(control, CONTROL, I, next == NULL);

  if (type & _DEV_DMA_SRC_REG) {
    phys->srcaddr = api->src.reg.addr;
    control = pl080_ins_val(control, CONTROL, SI, 0);
    control = pl080_ins_val(control, CONTROL, S, 0);
    control = pl080_ins_val(control, CONTROL, SWIDTH, api->src.reg.width);
    control = pl080_ins_val(control, CONTROL, SBSIZE, api->src.reg.burst ? bit_ctz((uint16_t)api->src.reg.burst) : 7);
    control = pl080_ins_val(control, CONTROL, TRANSFERSIZE, api->src.reg.size + 1);
  } else {
    phys->srcaddr = api->src.mem.addr;
    control = pl080_ins_val(control, CONTROL, SI, api->src.mem.inc != DEV_DMA_INC_0_UNIT);
    control = pl080_ins_val(control, CONTROL, S, 0);
    control = pl080_ins_val(control, CONTROL, SWIDTH, api->src.mem.width);
    control = pl080_ins_val(control, CONTROL, SBSIZE, 0);
    control = pl080_ins_val(control, CONTROL, TRANSFERSIZE, api->src.mem.size + 1);
  }

  if (type & _DEV_DMA_DST_REG) {
    phys->destaddr = api->dst.reg.addr;
    control = pl080_ins_val(control, CONTROL, DI, 0);
    control = pl080_ins_val(control, CONTROL, D, 0);
    control = pl080_ins_val(control, CONTROL, DWIDTH, api->src.reg.width);
    control = pl080_ins_val(control, CONTROL, DBSIZE, api->dst.reg.burst ? bit_ctz((uint16_t)api->dst.reg.burst) : 7);
  } else {
    phys->destaddr = api->dst.mem.addr;
    control = pl080_ins_val(control, CONTROL, DI, api->dst.mem.inc != DEV_DMA_INC_0_UNIT);
    control = pl080_ins_val(control, CONTROL, D, 0);
    control = pl080_ins_val(control, CONTROL, DWIDTH, api->src.mem.width);
    control = pl080_ins_val(control, CONTROL, DBSIZE, 0);
  }

  phys->control = control;
}

static
void pl080_channel_continuous_next(struct pl080_pv_s *pv, uint8_t chan)
{
  struct pl080_channel_s *c = &pv->channel[chan];
  struct dev_dma_rq_s *rq = pv->channel[chan].running;

  struct pl080_desc_s phys;
  pl080_descriptor_serialize(&phys, rq->type, &rq->desc[c->desc_index], NULL);
  pl080_chan_set32(pv->base, chan, SRCADDR, phys.srcaddr);
  pl080_chan_set32(pv->base, chan, DESTADDR, phys.destaddr);
  pl080_chan_set32(pv->base, chan, LLI, phys.lli);
  pl080_chan_set32(pv->base, chan, CONTROL, phys.control);
  pl080_chan_set32(pv->base, chan, CONFIGURATION, c->config);

  logk_debug("%d At %3d S %08x D %08x L %08x CN %08x CF %08x",
             chan, c->desc_index,
             phys.srcaddr, phys.destaddr, phys.lli, phys.control, c->config);
}

static
void pl080_channel_continue(struct pl080_pv_s *pv, uint8_t chan)
{
  struct pl080_channel_s *c = &pv->channel[chan];
  struct pl080_desc_s *chain = c->chain;

  assert(chain);

  logk_debug("Channel %d start cfg 0x%08x", chan, c->config);

  pl080_chan_set32(pv->base, chan, SRCADDR, chain->srcaddr);
  pl080_chan_set32(pv->base, chan, DESTADDR, chain->destaddr);
  pl080_chan_set32(pv->base, chan, LLI, chain->lli);
  pl080_chan_set32(pv->base, chan, CONTROL, chain->control);
  pl080_chan_set32(pv->base, chan, CONFIGURATION, c->config);
}

static
void pl080_channel_done(struct pl080_pv_s *pv, uint8_t chan, error_t err)
{
  struct dev_dma_rq_s *rq = pv->channel[chan].running;
  uint32_t src, dst, lli, control, config;

  pl080_set32(pv->base, INTERRCLR, bit(chan));
  pl080_set32(pv->base, INTTCCLEAR, bit(chan));

  src = pl080_chan_get32(pv->base, chan, SRCADDR);
  dst = pl080_chan_get32(pv->base, chan, DESTADDR);
  lli = pl080_chan_get32(pv->base, chan, LLI);
  control = pl080_chan_get32(pv->base, chan, CONTROL);
  config = pl080_chan_get32(pv->base, chan, CONFIGURATION);
    
  pl080_chan_set32(pv->base, chan, CONFIGURATION, 0);

  logk_debug("Channel %d done %d", chan, err);
  logk_debug("%d At end S %08x D %08x L %08x CN %08x CF %08x",
             chan,
             src, dst, lli, control, config);

  if (!err && rq && rq->loop_count_m1) {
    rq->loop_count_m1--;
    pl080_channel_continue(pv, chan);
    return;
  }

  if (!err && rq && (rq->type & _DEV_DMA_CONTINUOUS)) {
    bool_t go_on = rq->f_done(rq, pv->channel[chan].desc_index, 0);
    if (go_on) {
      logk_debug("Channel %d cont", chan);

      if (pv->channel[chan].desc_index == rq->desc_count_m1)
        pv->channel[chan].desc_index = 0;
      else
        pv->channel[chan].desc_index++;

      pl080_channel_continuous_next(pv, chan);
    } else {
      assert(!pv->channel[chan].chain);
      pv->channel[chan].running = NULL;
      rq->drv_pv = -1;
    }
    return;
  }
    
  if (pv->channel[chan].chain) {
    mem_free(pv->channel[chan].chain);
    pv->channel[chan].chain = NULL;
  }

  if (!rq)
    return;
    
  pv->channel[chan].running = NULL;
  rq->drv_pv = -1;
  rq->f_done(rq, 0, err);
}

static __attribute__((noinline))
uint8_t pl080_flow_control(enum dev_dma_rq_type_e type,
                           bool_t src_is_flow, bool_t dst_is_flow)
{
  static const uint8_t flow_mem2mem_dma = 0;
  static const uint8_t flow_mem2reg_dma = 1;
  static const uint8_t flow_reg2mem_dma = 2;
  static const uint8_t flow_reg2reg_dma = 3;
  static const uint8_t flow_reg2reg_dst = 4;
  static const uint8_t flow_mem2reg_dst = 5;
  static const uint8_t flow_reg2mem_src = 6;
  static const uint8_t flow_reg2reg_src = 7;

#define index(src_reg, dst_reg, src_flow, dst_flow) (((!!(src_reg)) << 3) | ((!!(dst_reg)) << 2) | ((!!(src_flow)) << 1) | ((!!(dst_flow)) << 0))
  static const uint8_t flow[] = {
    [index(0, 0, 0, 0)] = flow_mem2mem_dma,
    [index(0, 0, 0, 1)] = flow_mem2mem_dma,
    [index(0, 0, 1, 0)] = flow_mem2mem_dma,
    [index(0, 0, 1, 1)] = flow_mem2mem_dma,
    [index(0, 1, 0, 0)] = flow_mem2reg_dma,
    [index(0, 1, 0, 1)] = flow_mem2reg_dst,
    [index(0, 1, 1, 0)] = flow_mem2reg_dma,
    [index(0, 1, 1, 1)] = flow_mem2reg_dst,
    [index(1, 0, 0, 0)] = flow_reg2mem_dma,
    [index(1, 0, 0, 1)] = flow_reg2mem_dma,
    [index(1, 0, 1, 0)] = flow_reg2mem_src,
    [index(1, 0, 1, 1)] = flow_reg2mem_src,
    [index(1, 1, 0, 0)] = flow_reg2reg_dma,
    [index(1, 1, 0, 1)] = flow_reg2reg_dst,
    [index(1, 1, 1, 0)] = flow_reg2reg_src,
    [index(1, 1, 1, 1)] = flow_reg2reg_src,
  };

  return flow[index(type & _DEV_DMA_SRC_REG, type & _DEV_DMA_DST_REG, src_is_flow, dst_is_flow)];
#undef index
}

static
void pl080_channel_setup(struct pl080_pv_s *pv, uint8_t chan, struct dev_dma_rq_s *rq)
{
  assert(!bit_get(pl080_get32(pv->base, ENBLDCHNS), chan));

  pl080_set32(pv->base, INTTCCLEAR, bit(chan));
  pl080_set32(pv->base, INTERRCLR, bit(chan));

  uint8_t flow = pl080_flow_control(
                                    rq->type,
                                    ARM_PL080_LINK_IS_FLOW(rq->dev_link.src),
                                    ARM_PL080_LINK_IS_FLOW(rq->dev_link.dst));

  uint32_t config = 0;
  config = pl080_ins_val(config, CONFIGURATION, ITC, 1);
  config = pl080_ins_val(config, CONFIGURATION, IE, 1);
  config = pl080_ins_val(config, CONFIGURATION, FLOWCNTRL, flow);
  if (rq->type & _DEV_DMA_DST_REG)
    config = pl080_ins_val(config, CONFIGURATION, DESTPERIPHERAL, ARM_PL080_LINK_PERIPHERAL(rq->dev_link.dst));
  if (rq->type & _DEV_DMA_SRC_REG)
    config = pl080_ins_val(config, CONFIGURATION, SRCPERIPHERAL, ARM_PL080_LINK_PERIPHERAL(rq->dev_link.src));
  config = pl080_ins_val(config, CONFIGURATION, E, 1);

  pv->channel[chan].config = config;    

  if (rq->type & _DEV_DMA_CONTINUOUS) {
    pv->channel[chan].desc_index = 0;

    pl080_channel_continuous_next(pv, chan);

    return;
  }
    
  if (!rq->desc_count_m1 && !rq->loop_count_m1) {
    /* Simple case: single descriptor */
    struct pl080_desc_s phys;

    pl080_descriptor_serialize(&phys, rq->type, &rq->desc[0], NULL);

    pl080_chan_set32(pv->base, chan, SRCADDR, phys.srcaddr);
    pl080_chan_set32(pv->base, chan, DESTADDR, phys.destaddr);
    pl080_chan_set32(pv->base, chan, LLI, phys.lli);
    pl080_chan_set32(pv->base, chan, CONTROL, phys.control);

    pl080_chan_set32(pv->base, chan, CONFIGURATION, config);

    logk_debug("%d At beg S %08x D %08x L %08x CN %08x CF %08x",
               chan,
               phys.srcaddr, phys.destaddr, phys.lli, phys.control, config);

    return;
  }

  /* General case, we need a string of descriptors */

  struct pl080_desc_s *chain = mem_alloc(
                                         sizeof(struct pl080_desc_s) * (rq->desc_count_m1 + 1),
                                         mem_scope_sys);

  if (!chain) {
    pv->channel[chan].running = NULL;
    rq->drv_pv = -1;
    rq->f_done(rq, 0, -ENOMEM);
    return;
  }

  pv->channel[chan].chain = chain;

  for (size_t i = 0; i <= rq->desc_count_m1; ++i) {
    const struct dev_dma_desc_s *desc = &rq->desc[i];
    bool_t is_last = i == rq->desc_count_m1;
    struct pl080_desc_s *phys = &pv->channel[chan].chain[i];
    struct pl080_desc_s *next = is_last ? NULL : (phys+1);

    pl080_descriptor_serialize(phys, rq->type, desc, next);
  }

  pl080_channel_continue(pv, chan);
}

#if defined(CONFIG_DRIVER_ARM_PL080_IRQ_SEPARATE)

static DEV_IRQ_SRC_PROCESS(pl080_irq_tc)
{
  struct device_s *dev = ep->base.dev;
  struct pl080_pv_s *pv = dev->drv_pv;
 
  LOCK_SPIN_SCOPED(&dev->lock);

  uint8_t tc = pl080_get32(pv->base, INTTCSTATUS);

  for (uint8_t chan = 0; chan < pl080_channel_count(pv); ++chan) {
    if (bit_get(tc, chan))
      pl080_channel_done(pv, chan, 0);
  }
}

static DEV_IRQ_SRC_PROCESS(pl080_irq_err)
{
  struct device_s *dev = ep->base.dev;
  struct pl080_pv_s *pv = dev->drv_pv;
 
  LOCK_SPIN_SCOPED(&dev->lock);

  uint8_t error = pl080_get32(pv->base, INTERRORSTATUS);

  for (uint8_t chan = 0; chan < pl080_channel_count(pv); ++chan) {
    if (bit_get(error, chan))
      pl080_channel_done(pv, chan, -EIO);
  }
}

#else

static DEV_IRQ_SRC_PROCESS(pl080_irq_common)
{
  struct device_s *dev = ep->base.dev;
  struct pl080_pv_s *pv = dev->drv_pv;
 
  LOCK_SPIN_SCOPED(&dev->lock);

  uint8_t tc = pl080_get32(pv->base, INTTCSTATUS);
  uint8_t error = pl080_get32(pv->base, INTERRORSTATUS);

  for (uint8_t chan = 0; chan < pl080_channel_count(pv); ++chan) {
    if (bit_get(error, chan))
      pl080_channel_done(pv, chan, -EIO);
    else if (bit_get(tc, chan))
      pl080_channel_done(pv, chan, 0);
  }
}

#endif

#define pl080_use dev_use_generic

static DEVDMA_REQUEST(pl080_request)
{
  struct device_s *dev = accessor->dev;
  struct pl080_pv_s *pv = dev->drv_pv;
  uint8_t chan_count = pl080_channel_count(pv);
    
  va_list arg;
  va_start(arg, accessor);
  uint8_t used_channel_mask = 0;

  for (struct dev_dma_rq_s *rq = va_arg(arg, struct dev_dma_rq_s *);
       rq;
       rq = va_arg(arg, struct dev_dma_rq_s *))
    {
      if (!(rq->chan_mask & ~used_channel_mask)) {
        logk_debug("Cannot find free channel allocation, mask %x, used %x", rq->chan_mask, used_channel_mask);
        return -EBUSY;
      }

      for (size_t di = 0; di <= rq->desc_count_m1; ++di) {
        const struct dev_dma_desc_s *desc = &rq->desc[di];

        if ((rq->type & _DEV_DMA_DST_MEM)
            && desc->dst.mem.inc > DEV_DMA_INC_1_UNITS)
          return -ENOTSUP;

        if ((rq->type & _DEV_DMA_SRC_MEM)
            && desc->src.mem.inc > DEV_DMA_INC_1_UNITS)
          return -ENOTSUP;
      }
        
      used_channel_mask |= (rq->chan_mask & ~(rq->chan_mask-1));
    }
    
  va_end(arg);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  for (uint8_t chan = 0; chan < chan_count; ++chan) {
    if (bit_get(used_channel_mask, chan) && pv->channel[chan].running) {
      logk_debug("Some channels are busy used");
      return -EBUSY;
    }
  }

  va_start(arg, accessor);

  for (struct dev_dma_rq_s *rq = va_arg(arg, struct dev_dma_rq_s *);
       rq;
       rq = va_arg(arg, struct dev_dma_rq_s *))
    {
      logk_debug("RQ %p, %d desc", rq, rq->desc_count_m1+1);

      uint8_t chan = 0;
      for (uint8_t c = 0; c < chan_count; ++c) {
        if (!bit_get(rq->chan_mask, c))
          continue;
        if (pv->channel[c].running)
          continue;
        chan = c;
        goto found;
      }
      assert(!"Cannot allocate channel");
    found:
      rq->drv_pv = chan;
      pv->channel[chan].running = rq;

      pl080_channel_setup(pv, chan, rq);
    }
    
  va_end(arg);
    
  return 0;
}

static DEV_DMA_CANCEL(pl080_cancel)
{
  struct device_s *dev = accessor->dev;
  struct pl080_pv_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  struct pl080_channel_s *chan = NULL;
  uint8_t index;

  for (index = 0; index < pl080_channel_count(pv); ++index) {
    if (rq == pv->channel[index].running) {
      chan = &pv->channel[index];
      break;
    }
  }

  if (!chan) {
    logk_error("RQ %p not found", rq);
    return -ENOENT;
  }

  pl080_channel_halt(pv, index);
    
  uint32_t control = pl080_chan_get32(pv->base, index, CONTROL);
  uint32_t size_left = pl080_get(control, CONTROL, TRANSFERSIZE);

  rq->cancel.desc_idx = chan->desc_index;

  const struct dev_dma_desc_s *desc = &rq->desc[chan->desc_index];
  rq->cancel.size = desc->src.reg.size + 1 - size_left;

  chan->running = NULL;
  if (chan->chain) {
    mem_free(chan->chain);
    chan->chain = NULL;
  }
            
  return 0;
}

static DEV_DMA_GET_STATUS(pl080_get_status)
{
  struct device_s *dev = accessor->dev;
  struct pl080_pv_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  struct pl080_channel_s *chan = NULL;
  uint8_t index;

  for (index = 0; index < pl080_channel_count(pv); ++index) {
    if (rq == pv->channel[index].running) {
      chan = &pv->channel[index];
      break;
    }
  }

  if (!chan) {
    logk_error("RQ %p not found", rq);
    return -ENOENT;
  }

  if (!(rq->type & _DEV_DMA_CONTINUOUS)) {
    return -ENOTSUP;
  }
    
  status->src_addr = pl080_chan_get32(pv->base, index, SRCADDR);
  status->dst_addr = pl080_chan_get32(pv->base, index, DESTADDR);
    
  return 0;
}

static DEV_INIT(pl080_init)
{
  error_t err = 0;
  struct pl080_pv_s *pv;
  uintptr_t base;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &base, NULL);
  if (err)
    goto err_base;

  struct arm_identification_s id;
  arm_identification_read(&id, base);
  arm_identification_dump(&id);

  /* DDI0196G Table 3-28 DMACPeriphID3 Register bit assignments */
  /* Same format as PL081 (DDI0218E) */
  assert((id.pid0 & 0xffffe) == 0x41080);

  uint8_t rev = bit_get_mask(id.pid0, 20, 4);
  uint16_t pl_code = bit_get_mask(id.pid0, 0, 12);
  uint8_t chan_count = 2 << bit_get_mask(id.pid0, 24, 3);
  uint8_t ahb_count = bit_get(id.pid0, 27) + 1;
  uint32_t ahb_width = 32 << bit_get_mask(id.pid0, 28, 3);
  uint8_t req_count = bit_get(id.pid0, 31) ? 32 : 16;
    
  logk("PL%03x Rev. %d, %d channels, %d AHB masters, %d-bit bus, %d requestors",
       pl_code, rev, chan_count, ahb_count, ahb_width, req_count);

  pv = mem_alloc(sizeof(*pv) + chan_count * sizeof(*pv->channel), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  pv->base = base;

#if defined(CONFIG_DRIVER_ARM_PL080_IRQ_SEPARATE)
  device_irq_source_init(dev, &pv->irq_ep[ARM_PL080_IRQ_DMACINTERR], 1, &pl080_irq_err);
  device_irq_source_init(dev, &pv->irq_ep[ARM_PL080_IRQ_DMACINTTC], 1, &pl080_irq_tc);
#else
  device_irq_source_init(dev, &pv->irq_ep[ARM_PL080_IRQ_DMACINTR], 1, &pl080_irq_common);
#endif

  err = device_irq_source_link(dev, pv->irq_ep, ARM_PL080_IRQ_COUNT, -1);
  if (err)
    goto err_irq;
    
  pl080_set32(pv->base, CONFIGURATION, PL080_CONFIGURATION_E_MASK);
  for (uint8_t chan = 0; chan < chan_count; ++chan) {
    pv->channel[chan].running = NULL;
    pv->channel[chan].chain = NULL;
    pl080_chan_set32(pv->base, chan, CONFIGURATION, 0);
  }
    
  return 0;

 err_irq:
 err_mem:
  mem_free(pv);
 err_base:
  return err;
}

static DEV_CLEANUP(pl080_cleanup)
{
  struct pl080_pv_s *pv = dev->drv_pv;

  uint32_t chan_count = pl080_channel_count(pv);
    
  for (uint8_t chan = 0; chan < chan_count; ++chan)
    if (pv->channel[chan].running)
      return -EBUSY;

  for (uint8_t chan = 0; chan < chan_count; ++chan)
    if (pv->channel[chan].chain)
      mem_free(pv->channel[chan].chain);

  pl080_set32(pv->base, CONFIGURATION, 0);
  device_irq_source_unlink(dev, pv->irq_ep, ARM_PL080_IRQ_COUNT);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(arm_pl080_drv, 0, "ARM PL080 DMA", pl080,
               DRIVER_DMA_METHODS(pl080));

DRIVER_REGISTER(arm_pl080_drv);
