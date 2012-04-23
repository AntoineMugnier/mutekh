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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech

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

#ifdef CONFIG_CPU_SPARC_SINGLE_IRQ_EP
#define ICU_SPARC_MAX_VECTOR	1
#else
#define ICU_SPARC_MAX_VECTOR	15
#endif

struct sparc_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[ICU_SPARC_MAX_VECTOR];
#endif
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *sparc_icu_dev;

static CPU_INTERRUPT_HANDLER(sparc_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(sparc_icu_dev);
  struct sparc_dev_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_CPU_SPARC_SINGLE_IRQ_EP
    struct dev_irq_ep_s *sink = pv->sinks;
    int_fast16_t id = irq;

    sink->process(sink, &id);
#else
  if ( irq < ICU_SPARC_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
#endif
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(sparc_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(sparc_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
# endif
}

static DEVICU_GET_SINK(sparc_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct sparc_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= ICU_SPARC_MAX_VECTOR)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  sparc_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = sparc_icu_get_sink,
  .f_disable_sink = sparc_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = sparc_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************/

static DEV_CLEANUP(sparc_cleanup);
static DEV_INIT(sparc_init);

static const struct devenum_ident_s  sparc_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("cpu:sparc"),
#ifdef CONFIG_ARCH_GAISLER
  DEVENUM_GAISLER_ENTRY(0x1, 0x003), /* leon 3 */
  DEVENUM_GAISLER_ENTRY(0x1, 0x048), /* leon 4 */
#endif
  { 0 }
};

const struct driver_s  sparc_drv =
{
  .desc           = "Sparc processor",
  .id_table       = sparc_ids,

  .f_init         = sparc_init,
  .f_cleanup      = sparc_cleanup,

  .classes        = {
#ifdef CONFIG_DEVICE_IRQ
    &sparc_icu_drv,
#endif
    0
  }
};

REGISTER_DRIVER(sparc_drv);

static DEV_INIT(sparc_init)
{
  struct sparc_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

#ifdef CONFIG_ARCH_SMP
  struct dev_resource_s *res = device_res_get(dev, DEV_RES_ID, 0);
  if (!res)
    PRINTK_RET(-ENOENT, "sparc: device has no ID resource");

  if (res->id.major != cpu_id())
    PRINTK_RET(-EINVAL, "sparc: driver init must be executed on CPU with matching id");
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

  /* init sparc irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_SPARC_MAX_VECTOR, NULL);

  CPU_LOCAL_SET(sparc_icu_dev, dev);
  cpu_interrupt_sethandler(sparc_irq_handler);
#endif

  dev->drv = &sparc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(sparc_cleanup)
{
  struct sparc_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
# endif
  /* detach sparc irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_SPARC_MAX_VECTOR);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

