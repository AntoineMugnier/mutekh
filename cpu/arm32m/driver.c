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
#include <hexo/bit.h>

#include <cpu/arm32m/v7m.h>

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
      uint_fast8_t irq_no = irq - 16;
# if CONFIG_CPU_ARM32M_M_IRQ_COUNT != CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT
      uint_fast8_t sink_index = pv->sink_mapping[irq_no];
      if (sink_index >= CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT)
        break;
      struct dev_irq_sink_s *sink = pv->sinks + sink_index;
# else
      struct dev_irq_sink_s *sink = pv->sinks + irq_no;
# endif
      device_irq_sink_process(sink, 0);
      break;
    }
    default:
      break;
    }
}

static DEV_ICU_GET_SINK(arm_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if (id < CONFIG_CPU_ARM32M_M_IRQ_COUNT) {
# if CONFIG_CPU_ARM32M_M_IRQ_COUNT != CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT
    uint_fast8_t sink_index = pv->sink_mapping[id];
    if (sink_index >= CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT)
      return NULL;
    struct dev_irq_sink_s *sink = pv->sinks + sink_index;
    assert(sink->icu_pv == id);
    return sink;
# else
    return pv->sinks + id;
# endif
  }
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(arm_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t irq_no = sink->icu_pv;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      cpu_mem_write_32(ARMV7M_NVIC_ICER_ADDR(irq_no / 32),
                       ARMV7M_NVIC_ICER_CLRENA(irq_no % 32));
      return;
    case DEV_IRQ_SENSE_RISING_EDGE:
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      cpu_mem_write_32(ARMV7M_NVIC_ISER_ADDR(irq_no / 32),
                       ARMV7M_NVIC_ISER_SETENA(irq_no % 32));
    default:
      return;
    }
}

#define arm_icu_link device_icu_dummy_link

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
# if CONFIG_CPU_ARM32M_ARCH_VERSION == 7
  uint32_t type = cpu_mem_read_32(ARMV7M_MPU_TYPE_ADDR);
  uint_fast8_t r = 0;

#  ifdef CONFIG_CPU_ARM32M_MPU_STACK_GUARD
  r += ARM_M_STACK_GUARD_MPU_REGION_COUNT;
#  endif
#  ifdef CONFIG_CPU_ARM32M_MPU_NULL_PTR
  r++;
#  endif

  if (r > ARMV7M_MPU_TYPE_DREGION_GET(type))
    {
      printk("warning: not enough ARM MPU regions available (%u) "
        "to enable MPU_STACK_GUARD/MPU_NULL_PTR features.\n");
    }
  else
    {
#  ifdef CONFIG_CPU_ARM32M_MPU_NULL_PTR
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
#  endif

      /* enable MPU */
      cpu_mem_write_32(ARMV7M_MPU_CTRL_ADDR,
          ARMV7M_MPU_CTRL_ENABLE | ARMV7M_MPU_CTRL_PRIVDEFENA);
    }
# else
# error Unsupported arch revision
# endif
#endif
}

/************************************************************************/

static DEV_USE(arm_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      switch (accessor->api->class_)
        {
        case DRIVER_CLASS_TIMER:
          switch (accessor->number)
            {
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
            case 0:
            case 1:
              break;
#endif
#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
            case 2:
              break;
#endif
            default:
              return -ENOTSUP;
            }
          break;
        default:
          if (accessor->number > 0)
            return -ENOTSUP;
        }
      return 0;
    case DEV_USE_PUT_ACCESSOR:
      return 0;

#ifdef CONFIG_CPU_ARM32M_CLOCK
# if (defined (CONFIG_CPU_ARM32M_TIMER_SYSTICK) \
      || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)) &&    \
  defined(CONFIG_DEVICE_CLOCK_VARFREQ)
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct arm_dev_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
# if defined(CONFIG_DEVICE_IRQ) && defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK)
      pv->systick_rev += 2;
# endif
      return 0;
    }
# endif
    case DEV_USE_CLOCK_SINK_GATE_DONE:
      return 0;
#endif

    case DEV_USE_START:
    case DEV_USE_STOP:
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
      if (accessor->api->class_ == DRIVER_CLASS_TIMER)
        return arm_timer_systick_use(accessor, op);
#endif
      return 0;
    case DEV_USE_LAST_NUMBER:
      if (accessor->api->class_ == DRIVER_CLASS_TIMER)
        {
#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
          accessor->number = 2;
          return 0;
#elif defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK)
          accessor->number = 1;
          return 0;
#endif
        }
    default:
      return -ENOTSUP;
    }
}


enum {   /* same order as driver methods below */
#ifdef CONFIG_DEVICE_IRQ
  ARM32M_INITID_ICU,
#endif
#if defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK) || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)
  ARM32M_INITID_TIMER,
#endif
  ARM32M_INITID_CPU,
  ARM32M_INITID_CLOCK,
};

static DEV_INIT(arm_init)
{
  struct arm_dev_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_INIT_PARTIAL
  if (!device_init_is_partial(dev))
#endif
    {
      /* get processor device id specifed in resources */
      uintptr_t id = 0;
      if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
        PRINTK_RET(-ENOENT, "arm: device has no ID resource")
          ;

      /* allocate device private data */
      pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);
      if (pv == NULL)
        goto err;

      memset(pv, 0, sizeof(*pv));
      dev->drv_pv = pv;

      if (cpu_tree_node_init(&pv->node, id, dev))
        goto err_pv;
      if (cpu_tree_insert(&pv->node))
        {
          cpu_tree_node_cleanup(&pv->node);
          goto err_pv;
        }
#ifdef CONFIG_DEVICE_INIT_PARTIAL
      device_init_enable_api(dev, ARM32M_INITID_CPU);
#endif

#ifdef CONFIG_DEVICE_IRQ
      /* init arm irq sink endpoints */
      device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT,
                           arm_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_RISING_EDGE);

# if CONFIG_CPU_ARM32M_M_IRQ_COUNT != CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT
    {
      void *irq_map;
      error_t err = device_get_param_blob(dev, "irq_map", 0, &irq_map);
      if (err)
        return err;
      pv->sink_mapping = irq_map;

      for (uint_fast8_t irq_no = 0; irq_no < CONFIG_CPU_ARM32M_M_IRQ_COUNT; ++irq_no) {
        uint_fast8_t sink_index = pv->sink_mapping[irq_no];
        if (sink_index >= CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT)
          continue;
        pv->sinks[sink_index].icu_pv = irq_no;
      }
    }
# endif

      /* set processor interrupt handler */
      if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
        {
          CPU_LOCAL_SET(arm_icu_dev, dev);
          cpu_interrupt_sethandler(arm_irq_handler);
        }

# ifdef CONFIG_DEVICE_INIT_PARTIAL
      device_init_enable_api(dev, ARM32M_INITID_ICU);
# endif
#endif
    }

#ifdef CONFIG_DEVICE_INIT_PARTIAL
  if (!device_init_test_api(dev, ARM32M_INITID_CLOCK))
    {
# ifdef CONFIG_CPU_ARM32M_CLOCK
      /* postpone initialization of the clock input if the clock
         manager is not available yet */
      if (bit_get(cl_missing, DRIVER_CLASS_CMU))
        return -EAGAIN;
# endif
#endif

#ifdef CONFIG_CPU_ARM32M_CLOCK
      /* clock input init */
      dev_clock_sink_init(dev, &pv->clk_ep, DEV_CLOCK_EP_FREQ_NOTIFY |
                          DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC);
      if (dev_clock_sink_link(&pv->clk_ep, 0,
# if defined (CONFIG_CPU_ARM32M_TIMER_SYSTICK) || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)
                              &pv->freq /* need to get timer freq from CMU */
# else
                              NULL /* no timer, no need to store clock */
# endif
                              ))
        goto err_clock;
#elif defined (CONFIG_CPU_ARM32M_TIMER_SYSTICK) || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)
      if (device_get_res_freq(dev, &pv->freq, 0)) /* no CMU, get timer freq from resources */
        goto err_clock;
#endif

#ifdef CONFIG_DEVICE_INIT_PARTIAL
      device_init_enable_api(dev, ARM32M_INITID_CLOCK);
    }
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  pv->systick_start = 0;
  dev_rq_pqueue_init(&pv->systick_queue);
  pv->systick_period = CONFIG_CPU_ARM32M_TIMER_SYSTICK_PERIOD;
# ifdef CONFIG_DEVICE_IRQ
  pv->systick_rev = 1;
  /* enable systick in NVIC */
  cpu_mem_write_32(ARMV7M_NVIC_ISER_ADDR(0),
                   ARMV7M_NVIC_ISER_SETENA(15));
# endif

# ifdef CONFIG_DEVICE_INIT_PARTIAL
  device_init_enable_api(dev, ARM32M_INITID_TIMER);
# endif
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
  pv->dwt_cycnt_start = 0;
#endif

  return 0;

 err_clock:
#ifdef CONFIG_DEVICE_INIT_PARTIAL
  return -ENOENT;
#endif
 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
 err:
  return -EUNKNOWN;
}

static DEV_CLEANUP(arm_cleanup)
{
  struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  if (device_init_test_api(dev, ARM32M_INITID_TIMER))
    {
      if (pv->systick_start & 1)
        return -EBUSY;
    }

  cpu_mem_write_32(ARMV7M_SYST_CSR_ADDR, 0);
#endif

#ifdef CONFIG_CPU_ARM32M_CLOCK
  if (device_init_test_api(dev, ARM32M_INITID_CLOCK))
    dev_drv_clock_cleanup(dev, &pv->clk_ep);
#endif

  if (device_init_test_api(dev, ARM32M_INITID_CPU))
    {
      cpu_tree_remove(&pv->node);
      cpu_tree_node_cleanup(&pv->node);
    }

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(arm32m_drv,
#ifdef CONFIG_DEVICE_INIT_PARTIAL
                           DRIVER_FLAGS_NO_DEPEND | DRIVER_FLAGS_RETRY_INIT |
#endif
               DRIVER_FLAGS_EARLY_INIT, "Arm-m processor", arm,
#ifdef CONFIG_DEVICE_IRQ
               [ARM32M_INITID_ICU] = DRIVER_ICU_METHODS(arm_icu),
#endif
#if defined(CONFIG_CPU_ARM32M_TIMER_SYSTICK) || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)
               [ARM32M_INITID_TIMER] = DRIVER_TIMER_METHODS(arm_timer),
#endif
               [ARM32M_INITID_CPU] = DRIVER_CPU_METHODS(arm_cpu));

DRIVER_REGISTER(arm32m_drv,
                DEV_ENUM_FDTNAME_ENTRY("cpu:arm"));

