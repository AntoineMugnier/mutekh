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

#include <mutek/printk.h>
#include <device/driver.h>
#include <stdlib.h>
#include <device/class/dma.h>
#include "din.h"

#define DMA_TEST_CONTINOUS_ENABLE
#define DMA_TEST_LOOP_ENABLE

#define DMA_TEST_COUNT 1024
#define DMA_RQ_COUNT 4
#define DMA_MAX_DESC_COUNT 4
#define DMA_MAX_TRANSFER_SIZE (DMA_TEST_DIN_SIZE/DMA_MAX_DESC_COUNT/4)

enum dma_test_op_e
{
  DMA_TEST_MEM_TO_MEM,
  DMA_TEST_CONTINOUS,
  DMA_TEST_LOOP,
  DMA_TEST_MIXED,
  DMA_TEST_MIXED_ATOMIC,
};

struct dma_test_rq_s
{
  struct dev_dma_rq_s rq;                     
  struct dev_dma_desc_s desc[DMA_MAX_DESC_COUNT];
};

struct dma_test_ctx_s
{
  struct device_dma_s dma;
  uint16_t count[DMA_RQ_COUNT];
  uint32_t dout[DMA_RQ_COUNT][DMA_MAX_DESC_COUNT][DMA_MAX_TRANSFER_SIZE];
  struct dma_test_rq_s dma_rq[DMA_RQ_COUNT];
  struct kroutine_s  kr[DMA_RQ_COUNT];
  lock_t lock;
  uint32_t done_mask;
  enum dma_test_op_e next;
};

struct dma_test_ctx_s ctx;

static uint8_t dma_test_set_rand_inc(uint8_t width)
{
  switch (width)
    {
    case 0:
      return rand() % 4;
    case 1:
      return rand() % 2 + rand() % 2;
    case 2:
      return rand() % 2;
    default:
      abort();
    }
}

static uint8_t dma_test_get_inc_offset(uint8_t inc)
{
  return (inc == DEV_DMA_INC_4_UNITS ? 4 : inc);
}

static void dma_test_mem_to_mem_rq(struct dma_test_ctx_s * ctx, uint8_t idx);
static void dma_test_continous_rq(struct dma_test_ctx_s * ctx, uint8_t idx);
static void dma_test_loop_rq(struct dma_test_ctx_s * ctx, uint8_t idx);
static void dma_test_mixed_atomic_rq(struct dma_test_ctx_s * ctx);

static void dma_test_next_rq(struct dma_test_ctx_s * ctx, uint8_t i)
{
  switch(ctx->next)
    {
      case DMA_TEST_MEM_TO_MEM:
        dma_test_mem_to_mem_rq(ctx, i);
        break;
      case DMA_TEST_CONTINOUS:
        dma_test_continous_rq(ctx, i);
        break;
      case DMA_TEST_LOOP:
        dma_test_loop_rq(ctx, i);
        break;
      case DMA_TEST_MIXED:
      case DMA_TEST_MIXED_ATOMIC:
        switch(rand() % 4)
          {
#ifdef DMA_TEST_CONTINOUS_ENABLE
          case 0:
            dma_test_continous_rq(ctx, i);
            break;
#endif
#ifdef DMA_TEST_LOOP_ENABLE
          case 1:
            dma_test_loop_rq(ctx, i);
            break;
#endif
          default:
            dma_test_mem_to_mem_rq(ctx, i);
            break;
          }
        break;
      default:
        abort();
    }
}

static KROUTINE_EXEC(dma_test_process_next)
{
  switch(rand() % 16)
    {
#ifdef DMA_TEST_CONTINOUS_ENABLE
      case 0 ... 2:
        printk("Start CONTINOUS\n");
        ctx.next = DMA_TEST_CONTINOUS;
        break;
#endif
#ifdef DMA_TEST_LOOP_ENABLE
      case 3 ... 5:
        printk("Start LOOP\n");
        ctx.next = DMA_TEST_LOOP;
        break;
#endif
      case 6 ... 9:
        printk("Start MEM TO MEM\n");
        ctx.next = DMA_TEST_MEM_TO_MEM;
        break;
      case 10 ... 12:
        printk("Start MIXED\n");
        ctx.next = DMA_TEST_MIXED;
        break;
      default:
        printk("Start MIXED ATOMIC\n");
        ctx.next = DMA_TEST_MIXED_ATOMIC;
        break;
    }

  if (ctx.next == DMA_TEST_MIXED_ATOMIC)
    dma_test_mixed_atomic_rq(&ctx);
  else
    {
      for (uint16_t i = 0; i < DMA_RQ_COUNT; i++)
        {
          ctx.count[i] = 0;
          dma_test_next_rq(&ctx, i);
        }
    }
}

static KROUTINE_EXEC(dma_test_next)
{
  uint8_t i = kr - &ctx.kr[0];

  dma_test_next_rq(&ctx, i);
}

static void dma_test_check_data(struct dev_dma_desc_s *desc, uint8_t rq_idx, uint8_t desc_idx)
{
  uint8_t *src_next = (uint8_t *)din;
  uint8_t *dst_next = (uint8_t *)ctx.dout[rq_idx][desc_idx];

  struct dev_dma_rq_s *rq = (struct dev_dma_rq_s*)(ctx.dma_rq + rq_idx);

  for (uint16_t l = 0; l < rq->loop_count_m1 + 1; l++)
    {
      for (uint16_t i = 0; i < desc->src.mem.size; i++)
        {
          uint32_t src_data, dst_data;

          switch (desc->src.mem.width)
            {
            case 0:
              src_data = *src_next;
              dst_data = *dst_next;
              break;
            case 1:
              src_data = *((uint16_t *)src_next);
              dst_data = *((uint16_t *)dst_next);
              break;
            case 2:
              src_data = *((uint32_t *)src_next);
              dst_data = *((uint32_t *)dst_next);
              break;
            default:
              abort();
            }
  
          if (rq->type & _DEV_DMA_DST_MEM)
            dst_next += (1 << desc->src.mem.width) * DEV_DMA_GET_INC(desc->dst.mem.inc);
  
          if (rq->type & _DEV_DMA_SRC_MEM)
            src_next += (1 << desc->src.mem.width) * DEV_DMA_GET_INC(desc->src.mem.inc);
  
          if (src_data == dst_data)
            continue;
  
          printk("dma error:\n req number: %d\n req addr: 0x%x\n desc number: %d\n iteration: %d\n loop: %d\n src addr: 0x%08x  data: 0x%x \n dst addr: 0x%08x  data: 0x%x \n", rq_idx, rq, desc_idx, i, l, src_next, src_data, dst_next, dst_data);
  
          abort();
        }

      src_next = (uint8_t *)din + (1 << desc->src.mem.width) * desc->src.mem.stride;
      dst_next = (uint8_t *)ctx.dout[rq_idx][desc_idx] + (1 << desc->src.mem.width) * desc->dst.mem.stride;
    }

}

static DEV_DMA_CALLBACK(mem_to_mem_done)
{
  uint8_t idx = (struct dma_test_rq_s*)rq - &ctx.dma_rq[0];

  if (err)
    abort();

  for (uint8_t didx = 0; didx < rq->desc_count_m1 + 1; didx++)
    dma_test_check_data(rq->desc + didx, idx, didx);

  LOCK_SPIN_IRQ(&ctx.lock);

  if (ctx.count[idx] == DMA_TEST_COUNT)
    {
      ctx.done_mask |= (1 << idx);
      ctx.count[idx] = 0;

      printk("dma rq %d done \n", idx);

      if (ctx.done_mask != ((1 << DMA_RQ_COUNT) - 1))
        goto ret;

      ctx.done_mask = 0;

      kroutine_init_deferred(ctx.kr, &dma_test_process_next);
      kroutine_exec(ctx.kr);
    }
  else
    {
      /* Push next request */
      kroutine_init_deferred(&ctx.kr[idx], &dma_test_next);
      kroutine_exec(&ctx.kr[idx]);
      ctx.count[idx]++;
    }
ret:
  LOCK_RELEASE_IRQ(&ctx.lock);

  return 1;
}

static DEV_DMA_CALLBACK(continous_done)
{
  uint8_t idx = (struct dma_test_rq_s*)rq - &ctx.dma_rq[0];

  struct dev_dma_desc_s *desc = rq->desc + desc_id;

  dma_test_check_data(desc, idx, desc_id);
  memset((uint8_t *)ctx.dout[idx][desc_id], 0, 4 * DMA_MAX_TRANSFER_SIZE);

  uint8_t end = 1;

  LOCK_SPIN_IRQ(&ctx.lock);

  if (ctx.count[idx] == DMA_TEST_COUNT)
    {
      end = 0;
      ctx.done_mask |= (1 << idx);
      ctx.count[idx] = 0;

      printk("dma rq %d done \n", idx);

      if (ctx.done_mask != ((1 << DMA_RQ_COUNT) - 1))
        goto ret;

      ctx.done_mask = 0;

      kroutine_init_deferred(ctx.kr, &dma_test_process_next);
      kroutine_exec(ctx.kr);
    }
  else
    ctx.count[idx]++;

ret:
  LOCK_RELEASE_IRQ(&ctx.lock);

  return end;
}

static DEV_DMA_CALLBACK(loop_done)
{
  uint8_t idx = (struct dma_test_rq_s*)rq - &ctx.dma_rq[0];

  struct dev_dma_desc_s *desc = rq->desc + desc_id;

  dma_test_check_data(desc, idx, 0);

  LOCK_SPIN_IRQ(&ctx.lock);

  if (ctx.count[idx] == DMA_TEST_COUNT)
    {
      ctx.done_mask |= (1 << idx);
      ctx.count[idx] = 0;

      printk("dma rq %d done \n", idx);

      if (ctx.done_mask != ((1 << DMA_RQ_COUNT) - 1))
        goto ret;

      ctx.done_mask = 0;

      kroutine_init_deferred(ctx.kr, &dma_test_process_next);
      kroutine_exec(ctx.kr);
    }
  else
    {
      /* Push next request */
      kroutine_init_deferred(&ctx.kr[idx], &dma_test_next);
      kroutine_exec(&ctx.kr[idx]);
      ctx.count[idx]++;
    }

ret:
  LOCK_RELEASE_IRQ(&ctx.lock);

  return 1;
}


static void dma_test_set_rand_desc(struct dev_dma_desc_s *desc, uint8_t rq_idx, uint8_t desc_idx)
{
  struct dev_dma_rq_s *rq = &ctx.dma_rq[rq_idx].rq;

  memset((uint8_t *)ctx.dout[rq_idx][desc_idx], 0, 4 * DMA_MAX_TRANSFER_SIZE);

  switch (rq->type)
    {
    case DEV_DMA_REG_MEM_CONT:
      desc->src.reg.size = rand() % DMA_MAX_TRANSFER_SIZE;
      desc->src.reg.width = rand()%2 + rand()%2;
      desc->src.reg.addr = (uintptr_t)din;
      desc->src.reg.burst = DMA_MAX_TRANSFER_SIZE;
  
      desc->dst.mem.addr = (uintptr_t)ctx.dout[rq_idx][desc_idx];
      desc->dst.mem.inc = dma_test_set_rand_inc(desc->src.mem.width);

      if (desc->dst.mem.inc == 0)
        desc->dst.mem.inc = 1;
      break;

    case DEV_DMA_MEM_MEM:
      desc->src.mem.size = rand() % (DMA_MAX_TRANSFER_SIZE/2);
      desc->src.mem.width = rand()%2 + rand()%2;
      desc->src.mem.addr = (uintptr_t)din;
      desc->src.mem.stride = 0;
      desc->src.mem.inc = dma_test_set_rand_inc(desc->src.mem.width);
  
      desc->dst.mem.addr = (uintptr_t)ctx.dout[rq_idx][desc_idx];
      desc->dst.mem.inc = dma_test_set_rand_inc(desc->src.mem.width);
      desc->dst.mem.stride = 0;

      if (desc->dst.mem.inc == 0)
        desc->dst.mem.inc = 1;

      if (rq->loop_count_m1)
        {
          if (desc->src.mem.inc == 0)
            desc->src.mem.inc = 1;
  
          uint32_t size = desc->src.mem.size * dma_test_get_inc_offset(desc->src.mem.inc);
          desc->src.mem.stride = size + rand() % (DMA_MAX_TRANSFER_SIZE/2);

          size = desc->src.mem.size * dma_test_get_inc_offset(desc->dst.mem.inc);
          desc->dst.mem.stride = size + rand() % (DMA_MAX_TRANSFER_SIZE/2);
        }
      break;
    default:
      abort();
    }

/*
  if (rq->loop_count_m1)
    printk("size: %03d, sinc: %d, dinc: %d, width: %d, sstride: %03d, dstride: %03d\n",
        desc->src.mem.size + 1, desc->src.mem.inc, desc->dst.mem.inc, 
        desc->src.mem.width, desc->src.mem.stride, desc->dst.mem.stride);
  else
    printk("size: %03d, sinc: %d, dinc: %d, width: %d\n",
        desc->src.mem.size + 1, desc->src.mem.inc, desc->dst.mem.inc, desc->src.mem.width);
*/        
}

static void dma_test_set_continous_rq(struct dma_test_ctx_s * ctx, uint8_t idx)
{
  struct dev_dma_rq_s *rq = &ctx->dma_rq[idx].rq;

  rq->type = DEV_DMA_REG_MEM_CONT;
  rq->desc_count_m1 = 1 + rand() % (DMA_MAX_DESC_COUNT - 1);
  rq->loop_count_m1 = 0;
  rq->chan_mask = 0xFFFFFFFF;
  rq->f_done = continous_done;
  rq->cache_ptr = NULL;
  rq->dev_link.dst = 0;
  rq->dev_link.src = 0;

//  printk("Cont rq[%d]: %d, loop: %d\n", idx, rq->type, rq->loop_count_m1);

  for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++)
    dma_test_set_rand_desc(ctx->dma_rq[idx].desc + i, idx, i);
}

static void dma_test_continous_rq(struct dma_test_ctx_s * ctx, uint8_t idx)
{
  struct dev_dma_rq_s *rq = &ctx->dma_rq[idx].rq;

  dma_test_set_continous_rq(ctx, idx);

  if (DEVICE_OP(&ctx->dma, request, rq, NULL))
    abort();
}

static void dma_test_set_mem_to_mem_rq(struct dma_test_ctx_s * ctx, uint8_t idx)
{
  struct dev_dma_rq_s *rq = &ctx->dma_rq[idx].rq;

  rq->type = DEV_DMA_MEM_MEM;
  rq->desc_count_m1 = rand() % DMA_MAX_DESC_COUNT;
  rq->loop_count_m1 = 0;
  rq->chan_mask = 0xFFFFFFFF;
  rq->f_done = mem_to_mem_done;
  rq->cache_ptr = NULL;
  rq->dev_link.dst = 0;
  rq->dev_link.src = 0;

//  printk("M2M rq[%d]: %d, loop: %d\n", idx, rq->type, rq->loop_count_m1);

  for (uint8_t i = 0; i < rq->desc_count_m1 + 1; i++)
    dma_test_set_rand_desc(ctx->dma_rq[idx].desc + i, idx, i);
}

static void dma_test_mem_to_mem_rq(struct dma_test_ctx_s * ctx, uint8_t idx)
{
  struct dev_dma_rq_s *rq = &ctx->dma_rq[idx].rq;

  dma_test_set_mem_to_mem_rq(ctx, idx);

  if (DEVICE_OP(&ctx->dma, request, rq, NULL))
    abort();
}

static void dma_test_set_loop_rq(struct dma_test_ctx_s * ctx, uint8_t idx)
{
  struct dev_dma_rq_s *rq = &ctx->dma_rq[idx].rq;

  rq->type = DEV_DMA_MEM_MEM;
  rq->loop_count_m1 = 1 + rand() % (DMA_MAX_DESC_COUNT - 1);
  rq->desc_count_m1 = 0;
  rq->chan_mask = 0xFFFFFFFF;
  rq->f_done = loop_done;
  rq->cache_ptr = NULL;
  rq->dev_link.dst = 0;
  rq->dev_link.src = 0;

//  printk("Loop rq[%d]: %d, loop: %d\n", idx, rq->type, rq->loop_count_m1);

  dma_test_set_rand_desc(ctx->dma_rq[idx].desc, idx, 0);
}

static void dma_test_loop_rq(struct dma_test_ctx_s * ctx, uint8_t idx)
{
  struct dev_dma_rq_s *rq = &ctx->dma_rq[idx].rq;

  dma_test_set_loop_rq(ctx, idx);

  if (DEVICE_OP(&ctx->dma, request, rq, NULL))
    abort();
}

static void dma_test_mixed_atomic_rq(struct dma_test_ctx_s * ctx)
{
  if (DMA_RQ_COUNT > 4)
    abort();

  for (uint8_t i = 0; i < DMA_RQ_COUNT; i++)
    {
      ctx->count[i] = 0;

      switch(rand() % 8)
        {
#ifdef DMA_TEST_CONTINOUS_ENABLE
          case 0 ... 2:
           dma_test_set_continous_rq(ctx, i);
           break;
#endif
#ifdef DMA_TEST_LOOP_ENABLE
          case 3 ... 5:
           dma_test_set_loop_rq(ctx, i);
           break;
#endif
          default:
           dma_test_set_mem_to_mem_rq(ctx, i);
           break;
        }
    }

  struct dev_dma_rq_s *p[4];

  for (uint8_t i = 0; i < 4; i++)
    if (i  < DMA_RQ_COUNT)
      p[i] = &ctx->dma_rq[i].rq;
    else
      p[i] = NULL;

  if (DEVICE_OP(&ctx->dma, request, p[0], p[1], p[2], p[3], NULL))
   abort();
}

void app_start()
{
  printk("Start of DMA test\n");

  if (device_get_accessor_by_path(&ctx.dma.base, NULL, "dma", DRIVER_CLASS_DMA))
    printk("error: No DMA found matching \n");

  ctx.done_mask = 0;

  ctx.next = DMA_TEST_MEM_TO_MEM;
  kroutine_init_deferred(ctx.kr, &dma_test_process_next);
  kroutine_exec(ctx.kr);
}

