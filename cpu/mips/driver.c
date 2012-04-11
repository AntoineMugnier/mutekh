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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

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

struct mips_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_MIPS_MAX_VECTOR	6
  struct dev_irq_ep_s	sinks[ICU_MIPS_MAX_VECTOR];
#endif
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *mips_icu_dev;

static CPU_INTERRUPT_HANDLER(mips_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(mips_icu_dev);
  struct mips_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_MIPS_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(mips_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(mips_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status &= ~(1 << (icu_in_id + 10));
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif
}

static DEVICU_GET_SINK(mips_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct mips_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= ICU_MIPS_MAX_VECTOR)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status |= 1 << (icu_in_id + 10);
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  mips_icu_drv =
{
  .class_          = DEVICE_CLASS_ICU,
  .f_get_sink     = mips_icu_get_sink,
  .f_disable_sink = mips_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = mips_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************/

static DEV_CLEANUP(mips_cleanup);
static DEV_INIT(mips_init);

static const struct devenum_ident_s  mips_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("cpu:mips"),
  { 0 }
};

const struct driver_s  mips_drv =
{
  .desc           = "Mips processor",
  .id_table       = mips_ids,

  .f_init         = mips_init,
  .f_cleanup      = mips_cleanup,

  .classes        = {
#ifdef CONFIG_DEVICE_IRQ
    &mips_icu_drv,
#endif
    0
  }
};

REGISTER_DRIVER(mips_drv);

static DEV_INIT(mips_init)
{
  struct mips_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

#ifdef CONFIG_ARCH_SMP
  struct dev_resource_s *res = device_res_get(dev, DEV_RES_ID, 0);
  if (!res)
    PRINTK_RET(-ENOENT, "mips: device has no ID resource");

  if (res->id.major != cpu_id())
    PRINTK_RET(-EINVAL, "mips: driver init must be executed on CPU with matching id");
#endif

  /* FIXME allocation scope ? */
  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if ( pv == NULL )
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status |= 0xfc00;
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif

  /* init mips irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_MIPS_MAX_VECTOR, NULL);

  CPU_LOCAL_SET(mips_icu_dev, dev);
  cpu_interrupt_sethandler(mips_irq_handler);
#endif

  dev->drv = &mips_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(mips_cleanup)
{
  struct mips_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status &= ~0xfc00;
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif
  /* detach mips irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_MIPS_MAX_VECTOR);
#endif

  mem_free(pv);
}

