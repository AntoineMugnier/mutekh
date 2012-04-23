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

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

struct arm_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_ARM_MAX_VECTOR	1
  struct dev_irq_ep_s	sinks[ICU_ARM_MAX_VECTOR];
#endif
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

  if ( irq < ICU_ARM_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(arm_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(arm_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
# endif
}

static DEVICU_GET_SINK(arm_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= ICU_ARM_MAX_VECTOR)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  arm_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = arm_icu_get_sink,
  .f_disable_sink = arm_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = arm_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************/

static DEV_CLEANUP(arm_cleanup);
static DEV_INIT(arm_init);

static const struct devenum_ident_s  arm_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("cpu:arm"),
  { 0 }
};

const struct driver_s  arm_drv =
{
  .desc           = "Arm processor",
  .id_table       = arm_ids,

  .f_init         = arm_init,
  .f_cleanup      = arm_cleanup,

  .classes        = {
#ifdef CONFIG_DEVICE_IRQ
    &arm_icu_drv,
#endif
    0
  }
};

REGISTER_DRIVER(arm_drv);

static DEV_INIT(arm_init)
{
  struct arm_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

#ifdef CONFIG_ARCH_SMP
  struct dev_resource_s *res = device_res_get(dev, DEV_RES_ID, 0);
  if (!res)
    PRINTK_RET(-ENOENT, "arm: device has no ID resource");

  if (res->id.major != cpu_id())
    PRINTK_RET(-EINVAL, "arm: driver init must be executed on CPU with matching id");
#endif

  if (sizeof(*pv))
    {
      /* FIXME allocation scope ? */
      pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

      if ( pv == NULL )
        return -ENOMEM;

      memset(pv, 0, sizeof(*pv));
      dev->drv_pv = pv;
    }

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif

  /* init arm irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_ARM_MAX_VECTOR, NULL);

  CPU_LOCAL_SET(arm_icu_dev, dev);
  cpu_interrupt_sethandler(arm_irq_handler);
#endif

  dev->drv = &arm_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(arm_cleanup)
{
  struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
# endif
  /* detach arm irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_ARM_MAX_VECTOR);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

