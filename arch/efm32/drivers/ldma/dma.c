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

*/

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
#include <device/clock.h>

#include <arch/efm32/dma_request.h>
#include <arch/efm32/dma.h>

#define EFR32_LDMA_CHANNEL_MASK ((1 << CONFIG_DRIVER_EFR32_DMA_CHANNEL_COUNT) - 1)

enum efm32_dma_rq_state_e
{
  EFR32_DMA_DONE = 0,
  EFR32_DMA_ENQUEUED = 1,
  EFR32_DMA_ONGOING = 2,
};

struct efm32_dma_descriptor_s
{
  uint32_t ctrl;
  uint32_t src;
  uint32_t dst;
  uint32_t link;
};

struct efm32_dma_chan_state_s
{
  struct dev_dma_rq_s  *rq;
  uint8_t              first;
  uint8_t              last;
  /* Used in Ping-Pong mode */
  uint8_t              idx;
};

DRIVER_PV(struct efm32_dma_context_s
{
  /* base address of DMA */
  uintptr_t                         addr;

  struct dev_irq_src_s              irq_ep;
  dev_dma_queue_root_t              queue;
  struct dev_clock_sink_ep_s        clk_ep;
  uint8_t                           free_channel_mask;
  struct efm32_dma_chan_state_s     chan[CONFIG_DRIVER_EFR32_DMA_CHANNEL_COUNT];
  struct efm32_dma_descriptor_s     desc[CONFIG_DRIVER_EFR32_DMA_LINKED_LIST_SIZE];

  struct 
  {
    uint8_t                         free;
    uint8_t                         head;
  }list;
});

static inline uint8_t efm32_dma_get_src(struct dev_dma_rq_s * rq)
{
  switch(rq->type)
    {
    case DEV_DMA_MEM_MEM:
      return 0;
    case DEV_DMA_MEM_REG:  
    case DEV_DMA_MEM_REG_CONT:
      return rq->dev_link.dst & 0xFF;
    case DEV_DMA_REG_MEM:
    case DEV_DMA_REG_MEM_CONT:
      return rq->dev_link.src & 0xFF;
    default:
      abort();
    }
}

static inline uint8_t efm32_dma_get_signal(struct dev_dma_rq_s * rq)
{
  switch(rq->type)
    {
    case DEV_DMA_MEM_MEM:
      return 0;
    case DEV_DMA_MEM_REG:  
    case DEV_DMA_MEM_REG_CONT:
      return (rq->dev_link.dst >> 8) & 0xFF;
    case DEV_DMA_REG_MEM:
    case DEV_DMA_REG_MEM_CONT:
      return (rq->dev_link.src >> 8) & 0xFF;
    default:
      abort();
    }
}

static error_t efm32_dma_init_ctrl(struct dev_dma_rq_s * rq,
                                  struct dev_dma_desc_s * desc,
                                  struct efm32_dma_descriptor_s *chan_desc)
{
  chan_desc->ctrl = EFR32_LDMA_CH_CTRL_XFERCNT(desc->src.mem.size);
  chan_desc->src  = desc->src.mem.addr;
  chan_desc->dst  = desc->dst.mem.addr;

  EFR32_LDMA_CH_CTRL_SIZE_SETVAL(chan_desc->ctrl, desc->src.mem.width);

  uint16_t burst = 0;
  uint8_t src_inc = DEV_DMA_INC_0_UNIT;
  uint8_t dst_inc = DEV_DMA_INC_0_UNIT;

  switch(rq->type)
    {
    case DEV_DMA_MEM_MEM:
      src_inc = desc->src.mem.inc;
      dst_inc = desc->dst.mem.inc;
      break;
    case DEV_DMA_MEM_REG:  
    case DEV_DMA_MEM_REG_CONT:
      src_inc = desc->src.mem.inc;
      burst = desc->dst.reg.burst;
      break;
    case DEV_DMA_REG_MEM:
    case DEV_DMA_REG_MEM_CONT:
      dst_inc = desc->dst.mem.inc;
      burst = desc->src.reg.burst;
      break;
    default:
      return -ENOTSUP;
    }

  src_inc = (src_inc + 3) & 3;
  dst_inc = (dst_inc + 3) & 3;

  EFR32_LDMA_CH_CTRL_SRCINC_SETVAL(chan_desc->ctrl, src_inc);
  EFR32_LDMA_CH_CTRL_DSTINC_SETVAL(chan_desc->ctrl, dst_inc);

  if (efm32_dma_get_src(rq) == 0 || burst == 0)
    {
      /* Transfer all data at once */
      chan_desc->ctrl |= EFR32_LDMA_CH_CTRL_REQMODE;
      return 0;
    }

  switch (burst)
    {
    case 1 ... 4:
      burst -= 1;
      break;
    case 6:
      burst = 4;
      break;
    case 8 ... 1024:
      if (burst & (burst - 1))
        return -ENOTSUP;
      burst = bit_ctz(burst) + 2;
      break;
    default:
      return -ENOTSUP;
    }

  EFR32_LDMA_CH_CTRL_BLOCKSIZE_SETVAL(chan_desc->ctrl, burst);

  return 0;
}

static void efm32_dma_start(struct efm32_dma_context_s *pv,
                            struct dev_dma_rq_s * rq,
                            uint8_t  chan)
{
  uint32_t msk = endian_le32(1 << chan);
  uint32_t src = efm32_dma_get_src(rq);
  uint32_t signal = efm32_dma_get_signal(rq);
  uint32_t offset = src ? 0x04000000 : 0x06000000;

  pv->chan[chan].rq = rq;
  pv->free_channel_mask &= ~(1 << chan);

  uint32_t x = EFR32_LDMA_CH_REQSEL_SIGSEL(signal);
  EFR32_LDMA_CH_REQSEL_SOURCESEL_SETVAL(x, src);

  cpu_mem_write_32(pv->addr + EFR32_LDMA_CH_REQSEL_ADDR(chan), endian_le32(x));
  /* Triggering source */
  cpu_mem_write_32(pv->addr + EFR32_LDMA_REQDIS_ADDR + offset, msk);
  /* Enable interrupt */
  cpu_mem_write_32(pv->addr + EFR32_LDMA_IEN_ADDR + 0x06000000, msk);
}

static inline void efm32_dma_set_link_addr(struct efm32_dma_context_s *pv,
                                    struct efm32_dma_descriptor_s *desc,
                                    uint8_t index)
{
  desc->link = (uint32_t)(pv->desc + index % CONFIG_DRIVER_EFR32_DMA_LINKED_LIST_SIZE);
}

static inline uint8_t efm32_dma_list_get_offset(struct efm32_dma_context_s *pv,
                                         struct efm32_dma_descriptor_s *desc)
{
  return (uint8_t)(desc - pv->desc);
}

static inline struct efm32_dma_descriptor_s * efm32_dma_get_next_desc(struct efm32_dma_descriptor_s * dlist)
{
  return (struct efm32_dma_descriptor_s *)(dlist->link & 0xfffffffc);
}

static error_t efm32_dma_single_setup(struct efm32_dma_context_s *pv,
                                      struct dev_dma_rq_s * rq,
                                      uint8_t chan)
{
  struct efm32_dma_descriptor_s chan_desc;

  efm32_dma_init_ctrl(rq, &rq->desc[0], &chan_desc);

  chan_desc.ctrl |= EFR32_LDMA_CH_CTRL_DONEIFSEN;

  uint32_t addr = pv->addr + EFR32_LDMA_CH_CFG_ADDR(chan);

  cpu_mem_write_32(addr + 0x0, 0);
  cpu_mem_write_32(addr + 0x4, 0);
  cpu_mem_write_32(addr + 0x8, endian_le32(chan_desc.ctrl));
  cpu_mem_write_32(addr + 0xC, endian_le32(chan_desc.src));
  cpu_mem_write_32(addr + 0x10, endian_le32(chan_desc.dst));
  cpu_mem_write_32(addr + 0x14, endian_le32(0));

  efm32_dma_start(pv, rq, chan);

  cpu_mem_write_32(pv->addr + EFR32_LDMA_CHEN_ADDR + 0x06000000, endian_le32(1 << chan));

  if (!efm32_dma_get_src(rq))
    cpu_mem_write_32(pv->addr + EFR32_LDMA_SWREQ_ADDR, endian_le32(1 << chan));

  return 1;
}

static error_t efm32_dma_loop_setup(struct efm32_dma_context_s *pv,
                                    struct dev_dma_rq_s * rq,
                                    uint8_t chan)
{
  if (CONFIG_DRIVER_EFR32_DMA_LINKED_LIST_SIZE < 2 || rq->desc_count_m1)
    return -ENOTSUP;

  if (pv->list.free < 2)
    return 0;

  struct efm32_dma_descriptor_s *dlist = pv->desc + pv->list.head;
  struct efm32_dma_chan_state_s * state = pv->chan + chan;

  uint32_t addr = pv->addr + EFR32_LDMA_CH_CFG_ADDR(chan);
  struct dev_dma_desc_s *desc = rq->desc;

  cpu_mem_write_32(addr + 0, 0);
  cpu_mem_write_32(addr + 4, endian_le32(rq->loop_count_m1 - 1));
  cpu_mem_write_32(addr + 0x14, endian_le32((uint32_t)dlist));

  state->first = pv->list.head;

  /* First descriptor */

  efm32_dma_init_ctrl(rq, desc, dlist);

  dlist->link |= EFR32_LDMA_CH_LINK_LINK;

  if (!efm32_dma_get_src(rq))
    dlist->ctrl |= EFR32_LDMA_CH_CTRL_STRUCTREQ;

  /* Second descriptor */

  dlist = efm32_dma_get_next_desc(dlist);

  efm32_dma_init_ctrl(rq, desc, dlist);

  dlist->ctrl |= EFR32_LDMA_CH_CTRL_DONEIFSEN |
                 EFR32_LDMA_CH_CTRL_DECLOOPCNT |
                 EFR32_LDMA_CH_CTRL_SRCMODE |
                 EFR32_LDMA_CH_CTRL_DSTMODE |
                 EFR32_LDMA_CH_CTRL_DONEIFSEN;

  if (!efm32_dma_get_src(rq))
    dlist->ctrl |= EFR32_LDMA_CH_CTRL_STRUCTREQ;

  dlist->src = desc->src.mem.stride - ((desc->src.mem.size + 1) * DEV_DMA_GET_INC(desc->src.mem.inc));
  dlist->src *= (1 << desc->src.mem.width);

  dlist->dst = desc->dst.mem.stride - ((desc->src.mem.size + 1) * DEV_DMA_GET_INC(desc->dst.mem.inc));
  dlist->dst *= (1 << desc->src.mem.width);

  state->last = efm32_dma_list_get_offset(pv, dlist);

  struct efm32_dma_descriptor_s *l = efm32_dma_get_next_desc(dlist);
  pv->list.head = efm32_dma_list_get_offset(pv, l);

  efm32_dma_set_link_addr(pv, dlist, state->last);

  pv->list.free -= 2;

  efm32_dma_start(pv, rq, chan);

  cpu_mem_write_32(pv->addr + EFR32_LDMA_LINKLOAD_ADDR, EFR32_LDMA_LINKLOAD_LINKLOAD(chan));

  return 1;
}

static error_t efm32_dma_ping_pong_setup(struct efm32_dma_context_s *pv,
                                         struct dev_dma_rq_s * rq,
                                         uint8_t chan)
{

  if (!(rq->type & _DEV_DMA_DST_REG) && !(rq->type & _DEV_DMA_SRC_REG))
    return -EINVAL;

  if (rq->desc_count_m1 + 1 > pv->list.free)
    return 0;

  struct efm32_dma_descriptor_s *dlist = pv->desc + pv->list.head;
  struct efm32_dma_chan_state_s * state = pv->chan + chan;

  uint32_t addr = pv->addr + EFR32_LDMA_CH_CFG_ADDR(chan);

  cpu_mem_write_32(addr + 0, 0);
  cpu_mem_write_32(addr + 4, 0);
  cpu_mem_write_32(addr + 0x14, endian_le32((uint32_t)dlist));

  state->first = pv->list.head;
  state->idx = 0;

  for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++) 
    {
      efm32_dma_init_ctrl(rq, rq->desc + i, dlist);

      if (i == rq->desc_count_m1)
      /* Last transfer must point to the first one */
        {
          struct efm32_dma_descriptor_s *l = efm32_dma_get_next_desc(dlist);
          pv->list.head = efm32_dma_list_get_offset(pv, l);

          efm32_dma_set_link_addr(pv, dlist, state->first);
          state->last = efm32_dma_list_get_offset(pv, dlist);
        }
      
      dlist->link |= EFR32_LDMA_CH_LINK_LINK;
      dlist->ctrl |= EFR32_LDMA_CH_CTRL_DONEIFSEN;
#ifdef CONFIG_DRIVER_EFR32_DMA_TEST
      dlist->ctrl |= EFR32_LDMA_CH_CTRL_STRUCTREQ;
#endif
      dlist = efm32_dma_get_next_desc(dlist);
    }

  pv->list.free -= rq->desc_count_m1 + 1;

  efm32_dma_start(pv, rq, chan);

  cpu_mem_write_32(pv->addr + EFR32_LDMA_LINKLOAD_ADDR, EFR32_LDMA_LINKLOAD_LINKLOAD(chan));

  return 1;
}

static error_t efm32_dma_list_setup(struct efm32_dma_context_s *pv,
                                    struct dev_dma_rq_s * rq,
                                    uint8_t chan)
{
  struct efm32_dma_descriptor_s *dlist = pv->desc + pv->list.head;
  struct efm32_dma_chan_state_s * state = pv->chan + chan;

  uint32_t addr = pv->addr + EFR32_LDMA_CH_CFG_ADDR(chan);
  uint32_t x = (uint32_t)dlist;

  if (rq->desc_count_m1 + 1 > pv->list.free)
    return 0;

  cpu_mem_write_32(addr + 0, 0);
  cpu_mem_write_32(addr + 4, 0);
  cpu_mem_write_32(addr + 0x14, endian_le32(x));

  state->first = pv->list.head;

  for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++) 
    {
      efm32_dma_init_ctrl(rq, rq->desc + i, dlist);

      if (!efm32_dma_get_src(rq))
        dlist->ctrl |= EFR32_LDMA_CH_CTRL_STRUCTREQ;

      if (i == rq->desc_count_m1)
      /* Last transfer is not linked */
        {
          state->last = efm32_dma_list_get_offset(pv, dlist);
          dlist->link &= ~EFR32_LDMA_CH_LINK_LINK;
          dlist->ctrl |= EFR32_LDMA_CH_CTRL_DONEIFSEN;
        }
      else
        dlist->link |= EFR32_LDMA_CH_LINK_LINK;
      
      dlist = efm32_dma_get_next_desc(dlist);
    }

  pv->list.head = efm32_dma_list_get_offset(pv, dlist);
  pv->list.free -= rq->desc_count_m1 + 1;

  efm32_dma_start(pv, rq, chan);

  cpu_mem_write_32(pv->addr + EFR32_LDMA_LINKLOAD_ADDR, EFR32_LDMA_LINKLOAD_LINKLOAD(chan));

  return 1;
}

/* 
   Return 1 is request has been started. 
   Return 0 is request will be started later.
   Return ERROR is request is not supported or malformed.
*/

static error_t efm32_dma_process(struct efm32_dma_context_s *pv, struct dev_dma_rq_s * rq)
{
  if (!(rq->chan_mask & EFR32_LDMA_CHANNEL_MASK))
    return -ENOENT;

  if (rq->desc_count_m1 + 1 > CONFIG_DRIVER_EFR32_DMA_LINKED_LIST_SIZE ||
      rq->desc->src.mem.size + 1 > 4096)
    return -EINVAL;

  uint8_t chan_msk = rq->chan_mask & pv->free_channel_mask;

  if (chan_msk == 0)
    return 0;

  uint8_t chan = bit_ctz(chan_msk);

  assert(chan < CONFIG_DRIVER_EFR32_DMA_CHANNEL_COUNT);
  assert(pv->chan[chan].rq == NULL);

  if (rq->loop_count_m1)
  /* 2D copy */
    return efm32_dma_loop_setup(pv, rq, chan);
  if (rq->type & _DEV_DMA_CONTINUOUS)
  /* Ping-Pong mode */
    return efm32_dma_ping_pong_setup(pv, rq, chan);
  else if (rq->desc_count_m1)
  /* Scatter-gather */
    return efm32_dma_list_setup(pv, rq, chan);
  else
  /* Basic mode */
    return efm32_dma_single_setup(pv, rq, chan);
}

static DEVDMA_REQUEST(efm32_dma_request)
{
  struct device_s            *dev = accessor->dev;
  struct efm32_dma_context_s   *pv = dev->drv_pv;

  error_t start = 1;

  LOCK_SPIN_IRQ(&dev->lock);

#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (!dev->start_count)
    dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif

  va_list vl;
  va_start(vl, accessor);

  while(1)
  {
    struct dev_dma_rq_s *rq = va_arg(vl, struct dev_dma_rq_s *);

    if (rq == NULL)
      break;

    dev->start_count += DEVICE_START_COUNT_INC;

    if (start)
      start = efm32_dma_process(pv, rq);

    if (start < 0)
    /* An error has been detected */
      {
        rq->drv_pv = EFR32_DMA_DONE;
        break;
      }

    rq->drv_pv = start ? EFR32_DMA_ONGOING : EFR32_DMA_ENQUEUED;

    if (!start)
      dev_dma_queue_pushback(&pv->queue, rq);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return start < 0 ? start : 0;
}

static inline void
efm32_dev_dma_process_next(struct efm32_dma_context_s *pv)
{
  bool_t start = 1;

  GCT_FOREACH(dev_dma_queue, &pv->queue, r, {

    if (pv->free_channel_mask == 0)
     GCT_FOREACH_BREAK;

    assert(r->drv_pv == EFR32_DMA_ENQUEUED);

    if (start)
      start = efm32_dma_process(pv, r);

    if (start < 0)
      {
        r->drv_pv = EFR32_DMA_DONE;
        /* Request Callback here */
        r->f_done(r, 0, start);
        GCT_FOREACH_DROP;
      }

    if (start)
      {
        r->drv_pv = EFR32_DMA_ONGOING;
        GCT_FOREACH_DROP;
      }
    });
}

/* Release chains of linked list */
static void efm32_dma_release_list(struct efm32_dma_context_s *pv, uint8_t i)
{
  struct efm32_dma_chan_state_s * state = pv->chan + i;
  struct efm32_dma_descriptor_s * dlist = pv->desc + state->last;
  
  struct dev_dma_rq_s *rq = pv->chan[i].rq;

  if (rq->loop_count_m1)
    pv->list.free += 2;
  else
    pv->list.free += rq->desc_count_m1 + 1;

  dlist->link = (uint32_t)(pv->desc + pv->list.head);

  pv->list.head = state->first;
}

static inline void
efm32_dma_process_channel_irq(struct device_s *dev, uint8_t i)
{
  struct efm32_dma_context_s *pv = dev->drv_pv;
  struct dev_dma_rq_s *rq = pv->chan[i].rq;
  uint32_t msk = endian_le32(1 << i);

  assert(rq);

  uint8_t desc_id = 0;
  bool_t continous = (rq->type & _DEV_DMA_CONTINUOUS) ? 1 : 0;

  if (continous)
    {
      struct efm32_dma_chan_state_s * state = pv->chan + i;
      desc_id = state->idx;
      state->idx = desc_id == rq->desc_count_m1 ? 0 : desc_id + 1;
    }

  /* Request Callback here */
  bool_t run = rq->f_done(rq, desc_id, 0);

  if (continous && run)
    {
      /* Clear Interrupt Flag */
      cpu_mem_write_32(pv->addr + EFR32_LDMA_IFC_ADDR, msk);
      return;
    }

  rq->drv_pv = EFR32_DMA_DONE;

  /* Disable channel and interrupt */
  cpu_mem_write_32(pv->addr + EFR32_LDMA_IEN_ADDR + 0x04000000, msk);
  cpu_mem_write_32(pv->addr + EFR32_LDMA_CHEN_ADDR + 0x04000000, msk);
  cpu_mem_write_32(pv->addr + EFR32_LDMA_IFC_ADDR, msk);

  if (rq->desc_count_m1 || rq->loop_count_m1)
    efm32_dma_release_list(pv, i);

  pv->chan[i].rq = NULL;

  dev->start_count -= DEVICE_START_COUNT_INC;
  pv->free_channel_mask |= msk;

  efm32_dev_dma_process_next(pv);
}

static DEV_IRQ_SRC_PROCESS(efm32_dma_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_dma_context_s *pv = dev->drv_pv;
  
  lock_spin(&dev->lock);

  while(1)
  {
    uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_IF_ADDR));
    x &= endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_IEN_ADDR));

    if (!x)
      {
#ifdef CONFIG_DEVICE_CLOCK_GATING
        /* All queue are empty */
        if (!dev->start_count)
          dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
        break;
      }
 
    for (uint8_t i = 0; i < CONFIG_DRIVER_EFR32_DMA_CHANNEL_COUNT; i++) 
      {
        if (!(x & EFR32_LDMA_IF_DONE(i)))
          continue;

        efm32_dma_process_channel_irq(dev, i);
      }
  }
  
  lock_release(&dev->lock);
}

static error_t efm32_dev_dma_update_rq(struct efm32_dma_context_s *pv, uint8_t chan)
{
  struct efm32_dma_chan_state_s * state = pv->chan + chan;
  struct dev_dma_rq_s *rq = state->rq;
  uint32_t chdone = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_CHDONE_ADDR));

  uint32_t xfercnt = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_CH_CTRL_ADDR(chan)));
  xfercnt = EFR32_LDMA_CH_CTRL_XFERCNT_GET(xfercnt);

  rq->cancel.size = 0;
  rq->cancel.desc_idx = 0;

  if (rq->loop_count_m1)
  /* 2D copy */
    return -ENOTSUP;

  if (rq->type & _DEV_DMA_CONTINUOUS)
  /* Ping-Pong mode */
    {
      rq->cancel.desc_idx = state->idx;
      rq->cancel.size = rq->desc[state->idx].src.mem.size - xfercnt; 
    }
  else if (rq->desc_count_m1)
  /* Scatter-gather */
    {
      struct efm32_dma_descriptor_s * dlist = pv->desc + state->first;
      uint32_t link = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_CH_LINK_ADDR(chan)));
      link &= 0xfffffffc; 

      for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++) 
        {
          rq->cancel.desc_idx = i;

          if ((dlist->link & 0xfffffffc) == link)
          /* This is the last descriptor processed */
            {
              if (xfercnt || !(chdone & (1 << chan)))
                rq->cancel.size += rq->desc[i].src.mem.size - xfercnt;
              return 0;
            }

          rq->cancel.size += rq->desc[i].src.mem.size + 1;
          dlist = efm32_dma_get_next_desc(dlist);
        }
    }
  else
  /* Basic mode */
    rq->cancel.size = rq->desc[0].src.mem.size - xfercnt;

  return 0;
}

static DEV_DMA_GET_STATUS(efm32_dma_get_status)
{
  struct device_s *dev = accessor->dev;
  struct efm32_dma_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->drv_pv)
  {
    case EFR32_DMA_ENQUEUED:
      status->dst_addr = rq->desc->dst.mem.addr;
      status->src_addr = rq->desc->src.mem.addr;
      err = 0;
      break;

    case EFR32_DMA_DONE:
      err = -EBUSY;
      break;

    case EFR32_DMA_ONGOING:{
      /* Request is not terminated */
      uint8_t chan_msk = rq->chan_mask;

      while(1)
        {
          assert(chan_msk);

          uint8_t chan = bit_ctz(chan_msk);
          uint8_t msk  = (1 << chan);

          chan_msk &= ~msk;

          if (pv->chan[chan].rq != rq)
            continue;
            
          status->dst_addr = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_CH_DST_ADDR(chan)));
          status->src_addr = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_CH_SRC_ADDR(chan)));
          
          break;
        }}
      break;
    default:
      abort();
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}
static DEV_DMA_CANCEL(efm32_dma_cancel)
{
  struct device_s *dev = accessor->dev;
  struct efm32_dma_context_s *pv = dev->drv_pv;
  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->drv_pv)
  {
    case EFR32_DMA_ENQUEUED:
      err = 0;
      rq->cancel.size = 0;
      rq->cancel.desc_idx = 0;
      dev_dma_queue_remove(&pv->queue, rq);
      dev->start_count -= DEVICE_START_COUNT_INC;
    case EFR32_DMA_DONE:
      break;
    case EFR32_DMA_ONGOING:{
      /* Request is not terminated */
      uint8_t chan_msk = rq->chan_mask;

      while(1)
        {
          assert(chan_msk);

          uint8_t chan = bit_ctz(chan_msk);
          uint8_t msk  = (1 << chan);
          chan_msk &= ~msk;

          if (pv->chan[chan].rq != rq)
            continue;
            
          /* Disable channel */
          cpu_mem_write_32(pv->addr + EFR32_LDMA_CHEN_ADDR + 0x4000000, endian_le32(msk));
          /* Check interrupt */
          uint32_t done = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_IF_ADDR));

          if (done & msk)
          /* Interrupt is pending */
            break;

          cpu_mem_write_32(pv->addr + EFR32_LDMA_IEN_ADDR + 0x04000000, msk);
      
          err = efm32_dev_dma_update_rq(pv, chan);

          if (err)
            break;

          if (rq->desc_count_m1 || rq->loop_count_m1)
            efm32_dma_release_list(pv, chan);
      
          pv->chan[chan].rq = NULL;
      
          dev->start_count -= DEVICE_START_COUNT_INC;
          pv->free_channel_mask |= msk;
      
          efm32_dev_dma_process_next(pv);

          break;
        }}
      break;
    default:
      abort();
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(efm32_dma_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_dma_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_dma_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(efm32_dma_init)
{
  struct efm32_dma_context_s *pv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(struct efm32_dma_context_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(struct efm32_dma_context_s));

  pv->free_channel_mask = EFR32_LDMA_CHANNEL_MASK;
    
  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
    goto err_mem;

  /* Check number of channel */

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFR32_LDMA_STATUS_ADDR));

  x = EFR32_LDMA_STATUS_CHNUM_GET(x);
  assert(CONFIG_DRIVER_EFR32_DMA_CHANNEL_COUNT <= x);

  cpu_mem_write_32(pv->addr + EFR32_LDMA_IEN_ADDR, endian_le32(EFR32_LDMA_IEN_ERROR));
  cpu_mem_write_32(pv->addr + EFR32_LDMA_CHEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFR32_LDMA_CHDONE_ADDR, 0);

  dev_dma_queue_init(&pv->queue);

  /* Initialise linked list */

  pv->list.head = 0;
  pv->list.free = CONFIG_DRIVER_EFR32_DMA_LINKED_LIST_SIZE;

  for (uint8_t i = 0; i < CONFIG_DRIVER_EFR32_DMA_LINKED_LIST_SIZE; i++)
    efm32_dma_set_link_addr(pv, pv->desc + i, i + 1);

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_dma_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_clk;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  /* Release clock */
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

  err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
  err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_dma_cleanup)
{
  struct efm32_dma_context_s	*pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  dev_dma_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_dma_drv, 0, "EFM32 DMA", efm32_dma,
               DRIVER_DMA_METHODS(efm32_dma));

DRIVER_REGISTER(efm32_dma_drv);

