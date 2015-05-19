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

    Copyright (c) 2013 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <cpu/arm_v7m.h>

#include "driver.h"

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *arm_icu_dev;

static CPU_INTERRUPT_HANDLER(arm_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(arm_icu_dev);
  struct arm_dev_private_s  *pv = dev->drv_pv;

  switch (irq)
    {
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
    case 15: /* systick */
      arm_timer_systick_irq(dev);
      break;
#endif

    case 16 ... 16+CONFIG_CPU_ARM32M_M_IRQ_COUNT-1: {
      struct dev_irq_ep_s *sink = pv->sinks + irq - 16;
      int_fast16_t id = 0;

      sink->process(sink, &id);
      break;
    }
    default:
      break;
    }
}

static DEV_ICU_GET_ENDPOINT(arm_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < CONFIG_CPU_ARM32M_M_IRQ_COUNT)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEV_ICU_ENABLE_IRQ(arm_icu_enable_irq)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  /* inputs are single wire, logical irq id must be 0 */
  if (irq_id > 0)
    return 0;

  /* configure NVIC */
  cpu_mem_write_32(ARMV7M_NVIC_ISER_ADDR(icu_in_id / 32),
    ARMV7M_NVIC_ISER_SETENA(icu_in_id % 32));

  return 1;
}

static DEV_ICU_DISABLE_IRQ(arm_icu_disable_irq)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  /* configure NVIC */
  cpu_mem_write_32(ARMV7M_NVIC_ICER_ADDR(icu_in_id / 32),
    ARMV7M_NVIC_ICER_CLRENA(icu_in_id % 32));
}

const struct driver_icu_s  arm_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = arm_icu_get_endpoint,
  .f_enable_irq    = arm_icu_enable_irq,
  .f_disable_irq    = arm_icu_disable_irq,
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEV_CPU_REG_INIT(arm_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

  CPU_LOCAL_SET(cpu_device, dev);

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
  /* enable all types of fault */
  cpu_mem_write_32(ARMV7M_SHCSR_ADDR,
    ARMV7M_SHCSR_MEMFAULTENA | ARMV7M_SHCSR_BUSFAULTENA | ARMV7M_SHCSR_USGFAULTENA);
#endif

#if defined(CONFIG_CPU_ARM32M_MPU_STACK_GUARD) || defined(CONFIG_CPU_ARM32M_MPU_NULL_PTR)
  uint32_t type = cpu_mem_read_32(ARMV7M_MPU_TYPE_ADDR);
  uint_fast8_t r = 0;

# ifdef CONFIG_CPU_ARM32M_MPU_STACK_GUARD
  r += ARM_M_STACK_GUARD_MPU_REGION_COUNT;
# endif
# ifdef CONFIG_CPU_ARM32M_MPU_NULL_PTR
  r++;
# endif

  if (r > ARMV7M_MPU_TYPE_DREGION_GET(type))
    {
      printk("warning: not enough ARM MPU regions available (%u) "
        "to enable MPU_STACK_GUARD/MPU_NULL_PTR features.\n");
    }
  else
    {
# ifdef CONFIG_CPU_ARM32M_MPU_NULL_PTR
      cpu_mem_write_32(ARMV7M_MPU_RNR_ADDR,
          ARMV7M_MPU_RNR_REGION(r-1));

      /* Prevent memory access at address 0. 32 bytes sub-regions are
         enabled in order to cover the exception vector table. At
         least 64 bytes are covered. */
      cpu_mem_write_32(ARMV7M_MPU_RBAR_ADDR,
          ARMV7M_MPU_RBAR_ADDRESS(0));

      cpu_mem_write_32(ARMV7M_MPU_RASR_ADDR,
          ARMV7M_MPU_RASR_SIZE(7) | ARMV7M_MPU_RASR_ENABLE |
          ARMV7M_MPU_RASR_SRD(0xc0 |
            ((CONFIG_CPU_ARM32M_M_IRQ_COUNT < 8) << 2) |
            ((CONFIG_CPU_ARM32M_M_IRQ_COUNT < 16) << 3) |
            ((CONFIG_CPU_ARM32M_M_IRQ_COUNT < 24) << 4) |
            ((CONFIG_CPU_ARM32M_M_IRQ_COUNT < 32) << 5)));
# endif

      /* enable MPU */
      cpu_mem_write_32(ARMV7M_MPU_CTRL_ADDR,
          ARMV7M_MPU_CTRL_ENABLE | ARMV7M_MPU_CTRL_PRIVDEFENA);
    }

#endif
}

const struct driver_cpu_s  arm_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = arm_cpu_reg_init,
};

/************************************************************************/

static DEV_USE(arm_use)
{
  switch (accessor->api->class_)
    {
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
    case DRIVER_CLASS_TIMER:
      return arm_timer_systick_use(accessor, op);
#endif
      break;

    case DRIVER_CLASS_CPU:
    case DRIVER_CLASS_ICU:
      if (accessor->number > 0)
        break;
      switch (op)
        {
        case DEV_USE_GET_ACCESSOR:
        case DEV_USE_PUT_ACCESSOR:
          return 0;
        default:
          break;
        }
    default:
      break;
    }

  return -ENOTSUP;
}

static DEV_CLEANUP(arm_cleanup);
static DEV_INIT(arm_init);

static const struct dev_enum_ident_s  arm_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEV_ENUM_FDTNAME_ENTRY("cpu:arm"),
#endif
  { 0 }
};

const struct driver_s  arm32m_drv =
{
  .desc           = "Arm-m processor",
  .id_table       = arm_ids,

  .f_init         = arm_init,
  .f_cleanup      = arm_cleanup,
  .f_use          = arm_use,

  .classes        = {
    &arm_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &arm_icu_drv,
#endif
#if defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK) || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)
    &arm_m_timer_drv,
#endif
    0
  }
};

REGISTER_DRIVER(arm32m_drv);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(arm_clk_changed)
{
  struct device_s *dev = ep->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->freq = *freq;
# ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  pv->acc = *acc;
  pv->systick_rev += 2;
# endif
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

static DEV_INIT(arm_init)
{
  struct arm_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "arm: device has no ID resource")
      ;

  /* allocate device private data */
  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);
  if (pv == NULL)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_init(dev, &pv->clk_ep, &arm_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto no_clock;
  pv->freq = ckinfo.freq;
# ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  pv->acc = ckinfo.acc;
# endif

  if (dev_clock_sink_hold(&pv->clk_ep, 0))
    goto err_clku;

 no_clock:
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    pv->freq = DEV_FREQ_INVALID;
# ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  pv->acc = DEV_FREQ_ACC_INVALID;
# endif
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  pv->systick_start = 0;
  dev_request_pqueue_init(&pv->systick_queue);
  pv->systick_period = CONFIG_CPU_ARM32M_TIMER_SYSTICK_PERIOD;
# ifdef CONFIG_DEVICE_IRQ
  pv->systick_rev = 1;
  /* enable systick in NVIC */
  cpu_mem_write_32(ARMV7M_NVIC_ISER_ADDR(0),
    ARMV7M_NVIC_ISER_SETENA(15));
# endif
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
  pv->dwt_cycnt_start = 0;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init arm irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_ARM32M_M_IRQ_COUNT,
                       DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_RISING_EDGE);

  /* set processor interrupt handler */
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(arm_icu_dev, dev);
      cpu_interrupt_sethandler(arm_irq_handler);
    }
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_clk;

  dev->drv = &arm32m_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(arm_cleanup)
{
  struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR, 0);
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* detach arm irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, CONFIG_CPU_ARM32M_M_IRQ_COUNT);
#endif

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

