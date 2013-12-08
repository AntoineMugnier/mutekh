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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2009

*/

#include <string.h>

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/local.h>
#include <hexo/iospace.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

struct arm_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[CONFIG_CPU_ARM_M_IRQ_COUNT];
#endif

  struct cpu_tree_s node;
};

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
    case 15: /* systick */
      break;

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

/************************************************************************
        Timer driver part
************************************************************************/

static DEVTIMER_REQUEST(arm_timer_request)
{
  return -ENOTSUP;
}

static DEVTIMER_START_STOP(arm_timer_start_stop)
{
  return -ENOTSUP;
}

static DEVTIMER_GET_VALUE(arm_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

  switch (tdev->number)
    {
    case 0: {          /* cycle counter */

#if CONFIG_CPU_ARM_ARCH_VERSION >= 7
      *value = cpu_mem_read_32(/* DWT_CYCCNT reg */ 0xe0001004);
      return 0;
#endif
        return -ENOTSUP;
    }

    case 1: {
#warning SYSTICK
      return 0;
    }

      default:
        return -ENOTSUP;
    }

  return 0;
}

static DEVTIMER_RESOLUTION(arm_timer_resolution)
{
  error_t err = 0;

  switch (tdev->number)
    {
    case 0: {          /* cycle counter */
#if CONFIG_CPU_ARM_ARCH_VERSION >= 7
      if (res)
        {
          if (*res != 0)
            err = -ENOTSUP;
          *res = 1;
        }

      if (max)
        *max = 0xffffffff;
      return 0;
#endif
      return -ENOTSUP;
    }

    default:
      return -ENOTSUP;
    }

  return err;
}

static const struct driver_timer_s  arm_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_request       = arm_timer_request,
  .f_start_stop    = arm_timer_start_stop,
  .f_get_value     = arm_timer_get_value,
  .f_resolution    = arm_timer_resolution,
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

const struct driver_s  arm_drv =
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
    &arm_timer_drv,
    0
  }
};

REGISTER_DRIVER(arm_drv);

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

#ifdef CONFIG_DEVICE_IRQ
  /* init arm irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_ARM_M_IRQ_COUNT);

  /* set processor interrupt handler */
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(arm_icu_dev, dev);
      cpu_interrupt_sethandler(arm_irq_handler);
    }
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_node;

  dev->drv = &arm_drv;
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

#ifdef CONFIG_DEVICE_IRQ
  /* detach arm irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, CONFIG_CPU_ARM_M_IRQ_COUNT);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

