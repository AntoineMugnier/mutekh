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

    Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

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

#include <cpu/arm32m/pl230_dma.h>
#include <cpu/arm32m/pl230_channel.h>
#include <arch/efm32/dma_request.h>
#include <arch/efm32/dma.h>

#if CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT > EFM32_DMA_CHAN_COUNT
# error too many dma channels
#endif

#define PL230_CHANNEL_SIZE 16
#define EFM32_DMA_CHANNEL_MASK ((1 << CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT) - 1)

enum efm32_dma_rq_state_e
{
  EFR32_DMA_DONE = 0,
  EFR32_DMA_ENQUEUED = 1,
  EFR32_DMA_ONGOING = 2,
};

struct efm32_dma_chan_state_s
{
  struct dev_dma_rq_s  *rq;
  uint8_t               didx;
  uint8_t               lidx;
  /* Used for double buffering */
  bool_t                alt;
};

struct efm32_dma_descriptor_s
{
  uint32_t src;
  uint32_t dst;
  uint32_t cfg;
  uint32_t unused;
};

DRIVER_PV(struct efm32_dma_context_s
{
  /* base address of DMA */
  uintptr_t                     addr;
  /* Primary channel memory region */
  void *                        primary;

  struct dev_irq_src_s          irq_ep;
  dev_dma_queue_root_t          queue;
  struct dev_clock_sink_ep_s    clk_ep;

  uint8_t                       free_channel_mask;
  struct efm32_dma_chan_state_s chan[CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT];
  struct efm32_dma_descriptor_s desc[CONFIG_DRIVER_EFM32_DMA_LINKED_LIST_SIZE];

  bool_t                        list_busy;
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

static uint8_t efm32_dma_get_inc(uint8_t inc, uint8_t width)
{
  uint16_t lkt[3] = {0x2103, 0xF213, 0xFF23};

  return (lkt[width] >> (inc << 2)) & 0xFF;
}

static uint32_t efm32_dma_get_end_address(struct dev_dma_desc_s * desc, bool_t src)
{
  uint8_t  inc  = src ? desc->src.mem.inc  : desc->dst.mem.inc;
  uint32_t addr = src ? desc->src.mem.addr : desc->dst.mem.addr;

  return addr + desc->src.mem.size * (1 << desc->src.mem.width) * DEV_DMA_GET_INC(inc);
}

static error_t efm32_dma_set_ctrl(struct dev_dma_rq_s * rq,
                                  struct dev_dma_desc_s * desc,
                                  struct efm32_dma_descriptor_s *ctrl, 
                                  uint32_t mode)
{
  size_t len = desc->src.mem.size;
  size_t burst;

  if (len + 1 > 1024)
    return -EINVAL;

  uint8_t width = desc->src.mem.width;

  if (width > 2)
    return -EINVAL;

  uint8_t src_inc, dst_inc;

  ctrl->src = desc->src.mem.addr;
  ctrl->dst = desc->dst.mem.addr;
  ctrl->cfg = DMA_CHANNEL_CFG_N_MINUS_1(len);

  DMA_CHANNEL_CFG_SRC_SIZE_SETVAL(ctrl->cfg, width);
  DMA_CHANNEL_CFG_DST_SIZE_SETVAL(ctrl->cfg, width);
  DMA_CHANNEL_CFG_CYCLE_CTR_SETVAL(ctrl->cfg, mode);

  switch(rq->type)
    {
    case DEV_DMA_MEM_MEM:
      DMA_CHANNEL_CFG_R_POWER_SET(ctrl->cfg, AFTER1);
      src_inc = efm32_dma_get_inc(desc->src.mem.inc, width);
      dst_inc = efm32_dma_get_inc(desc->dst.mem.inc, width);
      ctrl->src = efm32_dma_get_end_address(desc, 1);
      ctrl->dst = efm32_dma_get_end_address(desc, 0);
      break;
    case DEV_DMA_MEM_REG:  
    case DEV_DMA_MEM_REG_CONT:
      burst = desc->dst.reg.burst;
      if (burst & (burst - 1))
      /* Not a power of 2 */
        return -EINVAL;
      DMA_CHANNEL_CFG_R_POWER_SETVAL(ctrl->cfg, bit_ctz(burst));
      src_inc = efm32_dma_get_inc(desc->src.mem.inc, width);
      dst_inc = 3;
      ctrl->src = efm32_dma_get_end_address(desc, 1);
      break;
    case DEV_DMA_REG_MEM:
    case DEV_DMA_REG_MEM_CONT:
      burst = desc->src.reg.burst;
      if (burst & (burst - 1))
      /* Not a power of 2 */
        return -EINVAL;
      DMA_CHANNEL_CFG_R_POWER_SETVAL(ctrl->cfg, bit_ctz(burst));
      src_inc = 3;
      dst_inc = efm32_dma_get_inc(desc->dst.mem.inc, width);
      ctrl->dst = efm32_dma_get_end_address(desc, 0);
      break;
    default:
      return -EINVAL;
    }

  if (src_inc == 0xF || dst_inc == 0xF)
    return -EINVAL;

  DMA_CHANNEL_CFG_SRC_INC_SETVAL(ctrl->cfg, src_inc);
  DMA_CHANNEL_CFG_DST_INC_SETVAL(ctrl->cfg, dst_inc);

  return 0;
}

static void efm32_dma_start(struct efm32_dma_context_s *pv,
                            struct dev_dma_rq_s * rq,
                            uint8_t chan)
{
  uint8_t src = efm32_dma_get_src(rq);
  uint8_t signal = efm32_dma_get_signal(rq);

  uint32_t msk = endian_le32(1 << chan);

  pv->chan[chan].rq = rq;
  pv->free_channel_mask &= ~(1 << chan);

  if (!rq->loop_count_m1 && !chan)
    {
      cpu_mem_write_32(pv->addr + EFM32_DMA_RECT0_ADDR, 0);
      cpu_mem_write_32(pv->addr + EFM32_DMA_LOOP_ADDR(0), 0);
    }

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IEN_ADDR));
  x |= msk;
  cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(x));
  
  x = EFM32_DMA_CH_CTRL_SOURCESEL(src) | EFM32_DMA_CH_CTRL_SIGSEL(signal); 
  cpu_mem_write_32(pv->addr + EFM32_DMA_CH_CTRL_ADDR(chan), endian_le32(x));

  /* Clear alternate setting */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHALTC_ADDR, msk);

  /* Enable channel */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENS_ADDR, msk);

  if (src)
    /* Peripheral triggering source */
    cpu_mem_write_32(pv->addr + PL230_DMA_CHREQMASKC_ADDR, msk);
  else
    /* Software triggering source */
    {
      cpu_mem_write_32(pv->addr + PL230_DMA_CHREQMASKS_ADDR, msk);
      cpu_mem_write_32(pv->addr + PL230_DMA_CHSWREQ_ADDR, msk);
    }
}

static error_t efm32_dma_loop_setup(struct efm32_dma_context_s *pv,
                                    struct dev_dma_rq_s * rq,
                                    uint8_t chan)
{
  if (rq->desc_count_m1 ||
      rq->type != DEV_DMA_MEM_MEM)
    return -ENOTSUP;

  uintptr_t ctrladdr = (uintptr_t)pv->primary;

  ctrladdr += PL230_CHANNEL_SIZE * chan;

  uint32_t mode = DMA_CHANNEL_CFG_CYCLE_CTR_AUTOREQUEST;

  if (efm32_dma_get_src(rq))
    mode = DMA_CHANNEL_CFG_CYCLE_CTR_BASIC;

  struct efm32_dma_descriptor_s ctrl;
  struct dev_dma_desc_s *desc = rq->desc;

  error_t err = efm32_dma_set_ctrl(rq, desc, &ctrl, mode);

  if (err)
    return err;

  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_CFG_ADDR, endian_le32(ctrl.cfg));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_SRC_DATA_END_ADDR, endian_le32(ctrl.src));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_DST_DATA_END_ADDR, endian_le32(ctrl.dst));

  uint32_t width = EFM32_DMA_LOOP_WIDTH(desc->src.mem.size);
  uint32_t src_stride = desc->src.mem.stride * (1 << desc->src.mem.width);
  uint32_t dst_stride = desc->dst.mem.stride * (1 << desc->src.mem.width);
  uint32_t height = rq->loop_count_m1;

  cpu_mem_write_32(pv->addr + EFM32_DMA_LOOP_ADDR(0), endian_le32(width));

  uint32_t x = EFM32_DMA_RECT0_HEIGHT(height) |
               EFM32_DMA_RECT0_SRCSTRIDE(src_stride) |
               EFM32_DMA_RECT0_DSTSTRIDE(dst_stride);

  cpu_mem_write_32(pv->addr + EFM32_DMA_RECT0_ADDR, endian_le32(x));

  efm32_dma_start(pv, rq, chan);

  return 1;
}

static error_t efm32_dma_basic_setup(struct efm32_dma_context_s *pv,
                                     struct dev_dma_rq_s * rq,
                                     uint8_t chan)
{
  uintptr_t ctrladdr = (uintptr_t)pv->primary;

  ctrladdr += PL230_CHANNEL_SIZE * chan;

  uint32_t mode = DMA_CHANNEL_CFG_CYCLE_CTR_AUTOREQUEST;

  if (efm32_dma_get_src(rq))
    mode = DMA_CHANNEL_CFG_CYCLE_CTR_BASIC;

  struct efm32_dma_descriptor_s ctrl;

  error_t err = efm32_dma_set_ctrl(rq, rq->desc, &ctrl, mode);

  if (err)
    return err;

  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_CFG_ADDR, endian_le32(ctrl.cfg));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_SRC_DATA_END_ADDR, endian_le32(ctrl.src));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_DST_DATA_END_ADDR, endian_le32(ctrl.dst));

  efm32_dma_start(pv, rq, chan);

  return 1;
}

static error_t efm32_dma_ping_pong_desc_setup(struct efm32_dma_context_s *pv,
                                              struct dev_dma_rq_s * rq,
                                              uint8_t chan)
{
  struct efm32_dma_chan_state_s *state = pv->chan + chan;
  struct dev_dma_desc_s *desc = &rq->desc[state->didx];

  uintptr_t ctrladdr = PL230_CHANNEL_SIZE * chan;

  if (state->alt)
    ctrladdr += cpu_mem_read_32(pv->addr + PL230_DMA_ALTCTRLBASE_ADDR);
  else
    ctrladdr += (uintptr_t)pv->primary;

  state->alt ^= 1;

  struct efm32_dma_descriptor_s ctrl;
  uint32_t mode = DMA_CHANNEL_CFG_CYCLE_CTR_PINGPONG;

  error_t err = efm32_dma_set_ctrl(rq, desc, &ctrl, mode);

  if (err)
    return err;

  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_CFG_ADDR, endian_le32(ctrl.cfg));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_SRC_DATA_END_ADDR, endian_le32(ctrl.src));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_DST_DATA_END_ADDR, endian_le32(ctrl.dst));

  state->didx = state->didx == rq->desc_count_m1 ? 0 : state->didx + 1;

  return 0;
}

static error_t efm32_dma_ping_pong_setup(struct efm32_dma_context_s *pv,
                                         struct dev_dma_rq_s * rq,
                                         uint8_t chan)
{
  assert(rq->desc_count_m1);

  struct efm32_dma_chan_state_s *state = pv->chan + chan;

  state->didx = 0;
  state->lidx = 0;
  state->alt  = 0;

  /* Configure primary descriptor */
  error_t err = efm32_dma_ping_pong_desc_setup(pv, rq, chan);

  if (err)
    return err;

  err = efm32_dma_ping_pong_desc_setup(pv, rq, chan);

  if (err)
    return err;

  efm32_dma_start(pv, rq, chan);

  return 1;
}

static error_t efm32_dma_list_setup(struct efm32_dma_context_s *pv,
                                    struct dev_dma_rq_s * rq,
                                    uint8_t chan)
{
  if (pv->list_busy)
    return 0;

  uintptr_t ctrladdr = (uintptr_t)pv->primary + PL230_CHANNEL_SIZE * chan;

  struct dev_dma_desc_s *desc;
  struct efm32_dma_descriptor_s *ctrl;
  uint32_t mode;
  uint8_t src = efm32_dma_get_src(rq);

  /* Build table of transfer */

  for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++)
    {
      desc = rq->desc + i;
      ctrl = pv->desc + i;

      if (i < rq->desc_count_m1)
        mode = src ? DMA_CHANNEL_CFG_CYCLE_CTR_PER_SG_ALT : DMA_CHANNEL_CFG_CYCLE_CTR_MEM_SG_ALT;
      else
        mode = src ? DMA_CHANNEL_CFG_CYCLE_CTR_BASIC : DMA_CHANNEL_CFG_CYCLE_CTR_AUTOREQUEST;

      error_t err = efm32_dma_set_ctrl(rq, desc, ctrl, mode);

      if (err) 
        return err;
    }

  /* Configure primary data structure */

  uint32_t x = DMA_CHANNEL_CFG_R_POWER(AFTER4) |
               DMA_CHANNEL_CFG_N_MINUS_1(((rq->desc_count_m1 + 1) << 2) - 1) |
               DMA_CHANNEL_CFG_SRC_INC(WORD) |
               DMA_CHANNEL_CFG_DST_INC(WORD) |
               DMA_CHANNEL_CFG_SRC_SIZE(WORD) |
               DMA_CHANNEL_CFG_DST_SIZE(WORD);

  if (efm32_dma_get_src(rq))
    x |= DMA_CHANNEL_CFG_CYCLE_CTR(PER_SG_PRI);
  else
    x |= DMA_CHANNEL_CFG_CYCLE_CTR(MEM_SG_PRI);

  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_CFG_ADDR, endian_le32(x));

  uint32_t addr = (uint32_t)pv->desc + (rq->desc_count_m1 + 1) * PL230_CHANNEL_SIZE - 4;
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_SRC_DATA_END_ADDR, endian_le32(addr));

  addr = cpu_mem_read_32(pv->addr + PL230_DMA_ALTCTRLBASE_ADDR) + PL230_CHANNEL_SIZE * (chan + 1) - 4;
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_DST_DATA_END_ADDR, endian_le32(addr));

  efm32_dma_start(pv, rq, chan);

  pv->list_busy = 1;

  return 1;
}

/* 
   Return 1 is request has been started. 
   Return 0 is request will be started later.
   Return ERROR is request is not supported or malformed.
*/

static error_t efm32_dma_process(struct efm32_dma_context_s *pv, struct dev_dma_rq_s * rq)
{
  if (!(rq->chan_mask & EFM32_DMA_CHANNEL_MASK))
    return -ENOENT;

  if (rq->desc_count_m1 + 1 > CONFIG_DRIVER_EFM32_DMA_LINKED_LIST_SIZE)
    return -ENOTSUP;

  uint8_t chan_msk = rq->chan_mask & pv->free_channel_mask;

  if (chan_msk == 0)
    return 0;

  uint8_t chan = bit_ctz(chan_msk);

  if (rq->loop_count_m1 && chan)
  /* Only channel 0 can be used in loop mode */
    return 0;

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
    return efm32_dma_basic_setup(pv, rq, chan);
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

static inline void efm32_dev_dma_process_next(struct efm32_dma_context_s *pv)
{
  error_t start = 1;

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
       
static inline void
efm32_dev_dma_process_channel_irq(struct device_s *dev, uint8_t i)
{
  struct efm32_dma_context_s *pv = dev->drv_pv;
  struct dev_dma_rq_s *rq = pv->chan[i].rq;

  assert(rq);

  struct efm32_dma_chan_state_s *state = pv->chan + i;
  uint32_t msk = endian_le32(1 << i);

  uint8_t desc_id = 0;
  bool_t continous = (rq->type & _DEV_DMA_CONTINUOUS) ? 1 : 0;

  /* Clear Interrupt Flag */
  cpu_mem_write_32(pv->addr + EFM32_DMA_IFC_ADDR, endian_le32(EFM32_DMA_IFC_DONE(i)));

  if (continous)
    {
      desc_id = state->lidx;
  
      if (desc_id == rq->desc_count_m1)
        state->lidx = 0;
      else
        state->lidx++;
    }

  /* Request Callback here */
  bool_t run = rq->f_done(rq, desc_id, 0);

  if (continous && run)
    {
      efm32_dma_ping_pong_desc_setup(pv, rq, i);
#ifdef CONFIG_DRIVER_EFM32_DMA_TEST
      /* Simulate a Peripheral restart */
      cpu_mem_write_32(pv->addr + PL230_DMA_CHSWREQ_ADDR, msk);
#endif
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PL230_DMA_CHENS_ADDR));
      if (x & (1 << i))
        return;
      /* Restart occurs too late */
      rq->f_done(rq, desc_id, -EINVAL);
    }

  rq->drv_pv = EFR32_DMA_DONE;

  /* Disable channel */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENC_ADDR, msk);

  pv->chan[i].rq = NULL;

  if (!continous && rq->desc_count_m1)
    pv->list_busy = 0;

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
    uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IF_ADDR));

    assert((x & EFM32_DMA_IF_ERR) == 0); 

    if (!x)
      {
#ifdef CONFIG_DEVICE_CLOCK_GATING
        if (!dev->start_count)
          dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
        break;
      }
 
    for (uint8_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 
      {
        if (!(x & EFM32_DMA_IF_DONE(i)))
          continue;

        efm32_dev_dma_process_channel_irq(dev, i);
      }
  }
  
  lock_release(&dev->lock);
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

static error_t efm32_dev_dma_update_rq(struct efm32_dma_context_s *pv, uint8_t chan)
{
  struct efm32_dma_chan_state_s * state = pv->chan + chan;
  struct dev_dma_rq_s *rq = state->rq;

  /* Channel must be disabled */
  assert((cpu_mem_read_32(pv->addr + PL230_DMA_CHENS_ADDR) & (1 << chan)) == 0);

  if (rq->loop_count_m1)
  /* 2D copy */
    return -ENOTSUP;

  uintptr_t priaddr = (uintptr_t)pv->primary + PL230_CHANNEL_SIZE * chan;
  uintptr_t altaddr = cpu_mem_read_32(pv->addr + PL230_DMA_ALTCTRLBASE_ADDR) + PL230_CHANNEL_SIZE * (chan);

  uint32_t prirem = cpu_mem_read_32(priaddr + DMA_CHANNEL_CFG_ADDR);
  prirem = DMA_CHANNEL_CFG_N_MINUS_1_GET(prirem);

  uint32_t altrem = cpu_mem_read_32(altaddr + DMA_CHANNEL_CFG_ADDR);
  altrem = DMA_CHANNEL_CFG_N_MINUS_1_GET(altrem);

  bool_t alt = (cpu_mem_read_32(pv->addr + PL230_DMA_CHALTS_ADDR) >> chan) & 1;

  rq->cancel.desc_idx = 0;
  rq->cancel.size = 0; 

  if (rq->type & _DEV_DMA_CONTINUOUS)
  /* Ping-Pong mode */
    {
      rq->cancel.desc_idx = state->lidx;
      rq->cancel.size = rq->desc[state->lidx].src.mem.size;
      rq->cancel.size -= alt ? altrem : prirem; 
    }
  else if (rq->desc_count_m1)
  /* Scatter-gather */
    {
      uint32_t priidx = (rq->desc_count_m1 + 1) - (prirem >> 2) - 1;

      if (prirem == 0 && alt)
      /* Primary channel not active and no remaining primary transfer */
        priidx++;

      if (alt)
        assert(!prirem || !((prirem + 1) & 0x3));

      if (priidx == 0)
        return 0;

      for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++)
        {
          rq->cancel.desc_idx = i;

          if (i == (priidx - 1))
            {
              if (altrem || alt)
                rq->cancel.size += rq->desc[i].src.mem.size - altrem;
              return 0;
            }

          rq->cancel.size += rq->desc[i].src.mem.size + 1;
        }
    }
  else
  /* Basic mode */
    rq->cancel.size = rq->desc[0].src.mem.size - prirem;

  return 0;
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
          cpu_mem_write_32(pv->addr + PL230_DMA_CHENC_ADDR, endian_le32(msk));
          /* Check interrupt */
          uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IF_ADDR));

          if (x & msk)
          /* Interrupt is pending */
            break;

          /* Disable irq */
          x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IEN_ADDR));
          cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(x & ~msk));
      
          err = efm32_dev_dma_update_rq(pv, chan);

          if (err)
            break;

          if (!(rq->type & _DEV_DMA_CONTINUOUS) && rq->desc_count_m1)
            pv->list_busy = 0;
      
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

static DEV_INIT(efm32_dma_init)
{
  struct efm32_dma_context_s *pv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(struct efm32_dma_context_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(struct efm32_dma_context_s));

  pv->free_channel_mask = EFM32_DMA_CHANNEL_MASK;

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
    goto err_mem;

  /* Check number of channel */

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PL230_DMA_STATUS_ADDR));

  x = PL230_DMA_STATUS_CHNUM_GET(x) + 1;
  assert(CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT <= x);

  size_t align, size;

  switch (x)
    {
      case 1:
        size = 0x10;
        break;
      case 2:
        size = 0x20;
        break;
      case 3 ... 4:
        size = 0x40;
        break;
      case 5 ... 8:
        size = 0x80;
        break;
      case 9 ... 16:
        size = 0x100;
        break;
      case 17 ... 32:
        size = 0x200;
        break;
      default:
       goto err_clk;
    }

  align = size * 2;

  /* Need alternate structure */
  size= align;

  pv->primary = mem_alloc_align(size, align, (mem_scope_sys));
  
  if (!pv->primary)
    goto err_clk;

  memset(pv->primary, 0, size);
  
  /* Enable DMA Controller */
  cpu_mem_write_32(pv->addr + PL230_DMA_CONFIG_ADDR, endian_le32(PL230_DMA_CONFIG_EN));

  /* Set primary and alternate control address */
  cpu_mem_write_32(pv->addr + PL230_DMA_CTRLBASE_ADDR, endian_le32((uint32_t)pv->primary));

  /* Select Primary and Alternate structure */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHALTC_ADDR, endian_le32(PL230_DMA_CHALTC_MASK));

  /* Disable Interrupts */
  cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(0));

  /* Disable Channels */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENC_ADDR, endian_le32(PL230_DMA_CHENC_MASK));

  dev_dma_queue_init(&pv->queue);

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_dma_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_prim;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  /* Release clock */
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

 err_prim:
  mem_free(pv->primary);
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

  mem_free(pv->primary);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_dma_drv, 0, "EFM32 DMA", efm32_dma,
               DRIVER_DMA_METHODS(efm32_dma));

DRIVER_REGISTER(efm32_dma_drv);

