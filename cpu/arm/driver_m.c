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

#include "driver_m.h"

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
#ifdef CONFIG_CPU_ARM_TIMER_SYSTICK
    case 15: /* systick */
      arm_timer_systick_irq(dev);
      break;
#endif

    case 16 ... 16+CONFIG_CPU_ARM_M_IRQ_COUNT-1: {
      struct dev_irq_ep_s *sink = pv->sinks + irq - 16;
      int_fast16_t id = 0;

      sink->process(sink, &id);
      break;
    }
    default:
      break;
    }
}

static DEVICU_GET_ENDPOINT(arm_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < CONFIG_CPU_ARM_M_IRQ_COUNT)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEVICU_ENABLE_IRQ(arm_icu_enable_irq)
{
  struct device_s *dev = idev->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  /* inputs are single wire, logical irq id must be 0 */
  if (irq_id > 0)
    return 0;

  /* configure NVIC */
  cpu_mem_write_32(0xe000e100 + 4 * (icu_in_id / 32), endian_le32(1 << (icu_in_id % 32)));

  return 1;
}

static DEVICU_DISABLE_IRQ(arm_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  /* configure NVIC */
  cpu_mem_write_32(0xe000e180 + 4 * (icu_in_id / 32), endian_le32(1 << (icu_in_id % 32)));
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

static DEVCPU_REG_INIT(arm_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

  CPU_LOCAL_SET(cpu_device, dev);
}

const struct driver_cpu_s  arm_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = arm_cpu_reg_init,
};

/************************************************************************/

static DEV_CLEANUP(arm_cleanup);
static DEV_INIT(arm_init);

static const struct devenum_ident_s  arm_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEVENUM_FDTNAME_ENTRY("cpu:arm"),
#endif
  { 0 }
};

const struct driver_s  arm_m_drv =
{
  .desc           = "Arm-m processor",
  .id_table       = arm_ids,

  .f_init         = arm_init,
  .f_cleanup      = arm_cleanup,

  .classes        = {
    &arm_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &arm_icu_drv,
#endif
#if defined(CONFIG_CPU_ARM_TIMER_SYSTICK) || defined(CONFIG_CPU_ARM_TIMER_DWTCYC)
    &arm_m_timer_drv,
#endif
    0
  }
};

REGISTER_DRIVER(arm_m_drv);

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

#ifdef CONFIG_CPU_ARM_TIMER_SYSTICK
  pv->systick_start = 0;
  dev_timer_queue_init(&pv->systick_queue);
  pv->systick_period = CONFIG_DEVICE_TIMER_DEFAULT_FREQ / 10; /* FIXME */
# ifdef CONFIG_DEVICE_IRQ
  /* enable systick in NVIC */
  cpu_mem_write_32(0xe000e100, endian_le32(1 << 15));
# endif
#endif

#ifdef CONFIG_CPU_ARM_TIMER_DWTCYC
  pv->dwt_cycnt_start = 0;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init arm irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_ARM_M_IRQ_COUNT,
                       DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_RISING_EDGE);

  /* set processor interrupt handler */
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(arm_icu_dev, dev);
      cpu_interrupt_sethandler(arm_irq_handler);
    }
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_node;

  dev->drv = &arm_m_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(arm_cleanup)
{
  struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_CPU_ARM_TIMER_SYSTICK
  cpu_mem_write_32(ARM_M_SYSTICK_CSR_ADDR, 0);
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* detach arm irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, CONFIG_CPU_ARM_M_IRQ_COUNT);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

