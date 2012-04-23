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

struct nios2_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_NIOS2_MAX_VECTOR	32
  struct dev_irq_ep_s	sinks[ICU_NIOS2_MAX_VECTOR];
#endif
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *nios2_icu_dev;

static CPU_INTERRUPT_HANDLER(nios2_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(nios2_icu_dev);
  struct nios2_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_NIOS2_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(nios2_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(nios2_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
  reg_t status = cpu_nios2_read_ctrl_reg(3);
  status &= ~(1 << icu_in_id);
  cpu_nios2_write_ctrl_reg(3, status);
# endif
}

static DEVICU_GET_SINK(nios2_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct nios2_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= ICU_NIOS2_MAX_VECTOR)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  reg_t status = cpu_nios2_read_ctrl_reg(3);
  status |= 1 << icu_in_id;
  cpu_nios2_write_ctrl_reg(3, status);
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  nios2_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = nios2_icu_get_sink,
  .f_disable_sink = nios2_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = nios2_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************/

static DEV_CLEANUP(nios2_cleanup);
static DEV_INIT(nios2_init);

static const struct devenum_ident_s  nios2_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("cpu:nios2"),
  { 0 }
};

const struct driver_s  nios2_drv =
{
  .desc           = "Nios II processor",
  .id_table       = nios2_ids,

  .f_init         = nios2_init,
  .f_cleanup      = nios2_cleanup,

  .classes        = {
#ifdef CONFIG_DEVICE_IRQ
    &nios2_icu_drv,
#endif
    0
  }
};

REGISTER_DRIVER(nios2_drv);

static DEV_INIT(nios2_init)
{
  struct nios2_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

#ifdef CONFIG_ARCH_SMP
  struct dev_resource_s *res = device_res_get(dev, DEV_RES_ID, 0);
  if (!res)
    PRINTK_RET(-ENOENT, "nios2: device has no ID resource");

  if (res->id.major != cpu_id())
    PRINTK_RET(-EINVAL, "nios2: driver init must be executed on CPU with matching id");
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
  cpu_nios2_write_ctrl_reg(3, -1);
# endif

  /* init nios2 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_NIOS2_MAX_VECTOR, NULL);

  CPU_LOCAL_SET(nios2_icu_dev, dev);
  cpu_interrupt_sethandler(nios2_irq_handler);
#endif

  dev->drv = &nios2_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(nios2_cleanup)
{
  struct nios2_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  cpu_nios2_write_ctrl_reg(3, 0);
# endif
  /* detach nios2 irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_NIOS2_MAX_VECTOR);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

