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

    Copyright (c) 2016 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <device/class/usbdev.h>
#include <device/usb/usb.h>

#include "dwc_efm32_regs.h"
#include "dwc.h"

DRIVER_PV(struct efm32_usbdev_private_s
{
  union{
    struct dev_usbdev_context_s usbdev_ctx;
    struct synopsys_usbdev_private_s synpv;
  };
  /* Clock Endpoint */
  struct dev_clock_sink_ep_s clk_ep[2];
  /* Interrupt end-point */
  struct dev_irq_src_s irq_eps;
});

static DEV_USBDEV_ENDPOINT(efm32_usbdev_endpoint)
{
  struct device_s *dev = ctx->dev;
  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  return synopsys_usbdev_endpoint(&pv->synpv, dir, address);
}

static DEV_USBDEV_CONFIG(efm32_usbdev_config)
{
  struct device_s *dev = ctx->dev;
  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  return synopsys_usbdev_config(dev, &pv->synpv, cfg);
}

static DEV_USBDEV_REQUEST(efm32_usbdev_transfer)
{
  struct device_s *dev = ctx->dev;
  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  return synopsys_usbdev_transfer(&pv->synpv, tr);
}

static DEV_IRQ_SRC_PROCESS(efm32_usb_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  uint32_t irq;

  lock_spin(&dev->lock);

  while(1)
    {
      irq = endian_le32(cpu_mem_read_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IF_ADDR));
      irq &= endian_le32(cpu_mem_read_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IEN_ADDR));

      if (irq & EFM32_USB_IF_VREGOSL) 
        {
          cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IFC_ADDR, EFM32_USB_IFC_VREGOSL);
          synopsys_usbdev_event(&pv->synpv, USBDEV_EVENT_DISCONNECT);
        }

      if (irq & EFM32_USB_IF_VREGOSH) 
        {
          cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IFC_ADDR, EFM32_USB_IFC_VREGOSH);
          synopsys_usbdev_event(&pv->synpv, USBDEV_EVENT_CONNECT);
        }

      if (!synopsys_usb_irq(&pv->synpv))
        break;
    }

  if (pv->synpv.event)
    synopsys_usbdev_abort_ep0(&pv->synpv);

  lock_release(&dev->lock);
}

static void efm32_usb_reset_device(struct device_s *dev)
{
  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  /* Enable USB pins */
  uint32_t x = EFM32_USB_ROUTE_PHYPEN | EFM32_USB_ROUTE_VBUSENPEN; 
  cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_ROUTE_ADDR, x);

  /* Disable and clear interrupts */
  cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IEN_ADDR, 0);
  cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IFC_ADDR, EFM32_USB_IFC_MASK);

  synopsys_usb_reset_device(&pv->synpv);

  /* Enable VREG Interrupt */
  cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_CTRL_ADDR, EFM32_USB_CTRL_VREGOSEN);
  cpu_mem_write_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_IEN_ADDR, EFM32_USB_IEN_MASK);
}

static DEV_USE(efm32_usbdev_use)
{
  struct device_accessor_s *accessor = param;
  struct device_s *dev = accessor->dev;

  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  switch (op)
    {
    case DEV_USE_START:
      if (dev->start_count == 0)
        synopsys_usbdev_start(&pv->synpv);
      break;
    case DEV_USE_STOP:
      if (dev->start_count == 0)
        {
          /* Hiding device during a control transfer blocks USB controller */
          efm32_usb_reset_device(dev); 
          /* Inform stack and all services */
          synopsys_usbdev_event(&pv->synpv, USBDEV_EVENT_STOP);
          synopsys_usbdev_stack_event(&pv->synpv);
        }
      break;
    default:
      return dev_use_generic(param, op);
    }
  return 0;
}

static const struct dev_usbdev_driver_ops_s efm32_usbdev_ops_s = {
  .f_transfer = efm32_usbdev_transfer,
  .f_config = efm32_usbdev_config,
  .f_alloc = synopsys_usbdev_alloc,
  .f_free = synopsys_usbdev_free,
  .f_endpoint = efm32_usbdev_endpoint
};

static DEV_INIT(efm32_usbdev_init);
static DEV_CLEANUP(efm32_usbdev_cleanup);

DRIVER_DECLARE(efm32_usbdev_drv, 0, "EFM32 USB", efm32_usbdev,
               DRIVER_USBDEV_METHODS(efm32_usbdev));

static DEV_INIT(efm32_usbdev_init)
{
  struct efm32_usbdev_private_s  *pv;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct efm32_usbdev_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  pv->synpv.addr = addr + EFM32_USB_SYNOPSYS_ADDR;

  dev->drv_pv = pv;

  if (dev_drv_clock_init(dev, &pv->clk_ep[0], 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
    goto err_mem;

  if (dev_drv_clock_init(dev, &pv->clk_ep[1], 1, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
    goto err_clk0;

  /* setup pinmux */
  if (device_iomux_setup(dev, ">vbusen", NULL, NULL, NULL))
    goto err_clk1;

  efm32_usb_reset_device(dev);

  if (usbdev_stack_init(dev, &pv->synpv.usbdev_ctx, SYNOPSYS_USBDEV_EP_MSK,
                        SYNOPSYS_USBDEV_EP_MSK, &efm32_usbdev_ops_s))
    goto err_clk1;

  /* Check if device is connected */
  uint32_t x = cpu_mem_read_32(pv->synpv.addr - EFM32_USB_SYNOPSYS_ADDR + EFM32_USB_STATUS_ADDR);
  if (endian_le32(x) & EFM32_USB_IF_VREGOSH)
    synopsys_usbdev_event(&pv->synpv, USBDEV_EVENT_CONNECT);
  
  device_irq_source_init(dev, &pv->irq_eps, 1, &efm32_usb_irq);

  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    goto err_clk1;

  dev->drv = &efm32_usbdev_drv;

  return 0;

err_clk1:
  dev_drv_clock_cleanup(dev, &pv->clk_ep[1]);
err_clk0:
  dev_drv_clock_cleanup(dev, &pv->clk_ep[0]);
err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_usbdev_cleanup)
{
  struct efm32_usbdev_private_s *pv = dev->drv_pv;

  dev_drv_clock_cleanup(dev, &pv->clk_ep[1]);
  dev_drv_clock_cleanup(dev, &pv->clk_ep[0]);

  device_irq_source_unlink(dev, &pv->irq_eps, 1);

  mem_free(pv);

  return 0;
}

