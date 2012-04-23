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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2011

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

struct lm32_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[CONFIG_CPU_LM32_IRQ_COUNT];
#endif
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *lm32_icu_dev;

static CPU_INTERRUPT_HANDLER(lm32_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(lm32_icu_dev);
  struct lm32_dev_private_s  *pv = dev->drv_pv;

  if ( irq < CONFIG_CPU_LM32_IRQ_COUNT ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(lm32_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(lm32_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
  reg_t status;
  asm volatile ("rcsr	%0, IM" : "=r" (status));
  status &= ~(1 << icu_in_id);
  asm volatile ("wcsr	IM, %0" :: "r" (status));
# endif
}

static DEVICU_GET_SINK(lm32_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= CONFIG_CPU_LM32_IRQ_COUNT)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  reg_t status;
  asm volatile ("rcsr	%0, IM" : "=r" (status));
  status |= 1 << icu_in_id;
  asm volatile ("wcsr	IM, %0" :: "r" (status));
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  lm32_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = lm32_icu_get_sink,
  .f_disable_sink = lm32_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = lm32_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************/

static DEV_CLEANUP(lm32_cleanup);
static DEV_INIT(lm32_init);

static const struct devenum_ident_s  lm32_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("cpu:lm32"),
  { 0 }
};

const struct driver_s  lm32_drv =
{
  .desc           = "LM32 processor",
  .id_table       = lm32_ids,

  .f_init         = lm32_init,
  .f_cleanup      = lm32_cleanup,

  .classes        = {
#ifdef CONFIG_DEVICE_IRQ
    &lm32_icu_drv,
#endif
    0
  }
};

REGISTER_DRIVER(lm32_drv);

static DEV_INIT(lm32_init)
{
  struct lm32_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

#ifdef CONFIG_ARCH_SMP
  struct dev_resource_s *res = device_res_get(dev, DEV_RES_ID, 0);
  if (!res)
    PRINTK_RET(-ENOENT, "lm32: device has no ID resource");

  if (res->id.major != cpu_id())
    PRINTK_RET(-EINVAL, "lm32: driver init must be executed on CPU with matching id");
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
  asm volatile ("wcsr	IM, %0" :: "r" ((1 << CONFIG_CPU_LM32_IRQ_COUNT)-1));

  reg_t status = cpu_lm32_mfc0(CPU_LM32_STATUS, 0);
  status |= 0xfc00;
  cpu_lm32_mtc0(CPU_LM32_STATUS, 0, status);
# endif

  /* init lm32 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_LM32_IRQ_COUNT, NULL);

  CPU_LOCAL_SET(lm32_icu_dev, dev);
  cpu_interrupt_sethandler(lm32_irq_handler);
#endif

  dev->drv = &lm32_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(lm32_cleanup)
{
  struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  asm volatile ("wcsr	IM, %0" :: "r" (0));
# endif
  /* detach lm32 irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, CONFIG_CPU_LM32_IRQ_COUNT);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

