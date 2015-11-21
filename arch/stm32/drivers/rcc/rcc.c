/*
    this file is part of mutekh.

    mutekh is free software; you can redistribute it and/or modify it
    under the terms of the gnu lesser general public license as
    published by the free software foundation; version 2.1 of the
    license.

    mutekh is distributed in the hope that it will be useful, but
    without any warranty; without even the implied warranty of
    merchantability or fitness for a particular purpose.  see the gnu
    lesser general public license for more details.

    you should have received a copy of the gnu lesser general public
    license along with mutekh; if not, write to the free software
    foundation, inc., 51 franklin street, fifth floor, boston, ma
    02110-1301 usa.

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/clock.h>

#include <arch/stm32f4xx_rcc.h>

#include <arch/stm32f4xx_memory_map.h>

#define STM32_CLOCK_SRC_AHB     0
#define STM32_CLOCK_SRC_APB1    1
#define STM32_CLOCK_SRC_APB2    2

#define STM32_CLOCK_NODE_HSI    0
#define STM32_CLOCK_NODE_PLL    1
#define STM32_CLOCK_NODE_HCLK   2
#define STM32_CLOCK_NODE_PCLK1  3
#define STM32_CLOCK_NODE_PCLK2  4

#define STM32_CLOCK_OSC_COUNT   1
#define STM32_CLOCK_SRC_COUNT   3
#define STM32_CLOCK_SIG_MAX     16

struct stm32_rcc_private_s
{
  /** device base address. */
  uintptr_t             addr;

  /** internal nodes. */
  struct {
    dev_clock_node_id_t src;
    dev_clock_node_id_t sink;
    dev_clock_frac_t    num;
    dev_clock_frac_t    denum;
  }                     signals[STM32_CLOCK_SIG_MAX];

  /** internal clock source end-points (i.e. oscillator). */
  struct dev_clock_ep_s oscs[STM32_CLOCK_OSC_COUNT];

  /** clock sink end-points. */
  struct dev_clock_ep_s srcs[STM32_CLOCK_SRC_COUNT];
};

static DEV_CLOCK_GATING(stm32_rcc_gating)
{
  /* FIXME: what should we really do here? */
  sink->u.sink.gating(sink, enable);
  return 0;
}

#define STM32_CLOCK_EDGE(u, v) ( (((v) & 0xff) << 8) | ((u) & 0xff) )

static
void stm32_rcc_update_source_freq(struct device_s *dev, uint_fast8_t src_id)
{
  assert(0 && "not yet implemented");
}

static
void stm32_rcc_update_sinks(struct device_s *dev, uint_fast8_t src_id)
{
  struct stm32_rcc_private_s *pv = dev->drv_pv;
  struct dev_clock_ep_s      *src = &pv->srcs[src_id];

  struct dev_clock_ep_s *iter;
  for (iter = src->u.src.sink_head; iter != NULL; iter = iter->u.sink.next)
    {
      iter->u.sink.config(iter);
    }
}

static DEV_CLOCK_SET_CONFIG(stm32_rcc_set_config)
{
  struct device_s            *dev = accessor->dev;
  struct stm32_rcc_private_s *pv  = dev->drv_pv;

  switch (STM32_CLOCK_EDGE(src_id, dst_id))
    {
    default:
      return -EINVAL;

    /* HSI -> PLL divided by M. */
    case STM32_CLOCK_EDGE(0, 1):
      STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, PLLSRC, HSI);
      STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, M, denum);
      break;

    /* PLL output P. */
    case STM32_CLOCK_EDGE(1, 2):
      STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, N, num);
      switch (denum)
        {
        default:
          return -EINVAL;

        case 2:
          STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, P, DIV_2);
          break;

        case 4:
          STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, P, DIV_4);
          break;

        case 6:
          STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, P, DIV_6);
          break;

        case 8:
          STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, P, DIV_8);
          break;
        }
      break;

    /* PLL output Q. */
    case STM32_CLOCK_EDGE(1, 3):
      STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, N, num);
      STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, PLLCFGR, Q, denum);
      break;

    /* PLL -> AHB divided by AHB prescaler. */
    case STM32_CLOCK_EDGE(2, 4):
      if (denum > 16)
        return -EINVAL;

      STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, CFGR, SW, PLL);
      if (denum == 1)
        STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, CFGR, HPRE, 0);
      else
        STM32F4xx_REG_FIELD_UPDATE_DEV(
          RCC,
          pv->addr,
          CFGR,
          HPRE,
          /* FIXME: does not work for prescaler > 16. */
          0x8 | ((__builtin_ctz(denum) >> 1) && 0x7)
        );

      stm32_rcc_update_source_freq(dev, 0 /* AHB */);
      stm32_rcc_update_sinks(dev, 0 /* AHB */);
      break;

    /* PLL -> APB1 divided by APB1 prescaler. */
    case STM32_CLOCK_EDGE(4, 5):
      if (num == denum)
        STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, CFGR, PPRE1, 0);
      else
        STM32F4xx_REG_FIELD_UPDATE_DEV(
          RCC,
          pv->addr,
          CFGR,
          PPRE1,
          0x4 | ((__builtin_ctz(denum) >> 1) && 0x3)
        );
      stm32_rcc_update_source_freq(dev, 1 /* APB1 */);
      stm32_rcc_update_sinks(dev, 1 /* APB1 */);
      break;

    /* PLL -> APB2 divided by APB2 prescaler. */
    case STM32_CLOCK_EDGE(4, 6):
      if (denum == 1)
        STM32F4xx_REG_FIELD_UPDATE_DEV(RCC, pv->addr, CFGR, PPRE2, 0);
      else
        STM32F4xx_REG_FIELD_UPDATE_DEV(
          RCC,
          pv->addr,
          CFGR,
          HPRE,
          0x4 | ((__builtin_ctz(denum) >> 1) && 0x3)
        );
      stm32_rcc_update_source_freq(dev, 2 /* APB2 */);
      stm32_rcc_update_sinks(dev, 2 /* APB2 */);
      break;
    }

  return 0;
}

#undef STM32_CLOCK_EDGE

static DEV_CLOCK_GET_ENDPOINT(stm32_rcc_get_endpoint)
{
  struct device_s            *dev = accessor->dev;
  struct stm32_rcc_private_s *pv  = dev->drv_pv;

  switch (type)
    {
    default:
      return NULL;

    case DEV_CLOCK_NODE_OSCILLATOR:
      if (node_id > STM32_CLOCK_OSC_COUNT-1)
        return NULL;
      return &pv->oscs[0];

    case DEV_CLOCK_NODE_EP_SOURCE:
      node_id -= 4; /* align on source range (4-6). */
      if (node_id > STM32_CLOCK_SRC_COUNT-1)
        return NULL;
      return &pv->srcs[node_id];
    }
}

static DEV_INIT(stm32_rcc_init);
static DEV_CLEANUP(stm32_rcc_cleanup);

#define stm32_rcc_use dev_use_generic

DRIVER_DECLARE(stm32_rcc_drv, 0, "STM32 RCC", stm32_rcc,
               DRIVER_CLOCK_METHODS(stm32_rcc));

DRIVER_REGISTER(stm32_rcc_drv);

static DEV_INIT(stm32_rcc_init)
{
  struct stm32_rcc_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* several inits and register config here. */

  /* initialize end-points. */
  dev_clock_source_init(dev, &pv->srcs[0]);
  dev_clock_source_init(dev, &pv->srcs[1]);
  dev_clock_source_init(dev, &pv->srcs[2]);

  /* initialize internal oscillator. */
  if (dev_clock_osc_init_by_id(dev, &pv->oscs[0], 0))
    goto err_mem;

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32_rcc_cleanup)
{
  struct stm32_rcc_private_s *pv = dev->drv_pv;
  mem_free(pv);
}

