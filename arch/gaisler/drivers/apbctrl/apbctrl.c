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


#include <hexo/types.h>
#include <hexo/endian.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/class/enum.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <arch/device_ids.h>

#include <string.h>

static DEVENUM_MATCH_DRIVER(apbctrl_match_driver)
{
  const struct devenum_ident_s *ident = drv->id_table;

  if (!ident)
    return 0;

  for ( ; ident->type != 0; ident++ )
    {
      if (ident->type != DEVENUM_TYPE_GAISLER)
        continue;

      const struct dev_resource_s *rp = device_res_get(dev, DEV_RES_PRODUCTID, 0);
      const struct dev_resource_s *rv = device_res_get(dev, DEV_RES_VENDORID, 0);

      if (rv && rp && rv->vendor.id == ident->grlib.vendor && rp->product.id == ident->grlib.device)
        return 1;
    }

  return 0;
}


#ifdef CONFIG_DEVICE_IRQ 

extern struct device_s *gaisler_icu;

static DEVENUM_GET_DEFAULT_ICU(apbctrl_get_default_icu)
{
  assert(dev->enum_dev == edev->dev);
  return gaisler_icu;
}


/* Traverse device tree to add irq resource between irqmp and processors. */

struct apbctrl_scan_cpu_irq_ctx_s
{
  struct device_s *irqmp;
  uint_fast8_t count;
};

static DEVICE_TREE_WALKER(apbctrl_scan_cpu_irq)
{
  struct apbctrl_scan_cpu_irq_ctx_s *ctx = priv;

  if (dev->node.flags & DEVICE_FLAG_CPU)
    {
      uintptr_t maj, min;
      if (device_res_get_uint(dev, DEV_RES_ID, 0, &maj, &min))
        return 0;

#ifndef CONFIG_ARCH_SMP
      if (maj > 0)
        return 0;
#endif
      device_res_add_irq(ctx->irqmp, maj, 0, 0, dev);
      ctx->count++;
    }

  return 0;
}
#endif


/* Enumerate devices using Gaisler APB pnp records. */

static void apbctrl_scan(struct device_s *dev, uintptr_t begin)
{
  uint_fast8_t i;

  for (i = 0; i < 16; i++)
    {
      const uint32_t *p = (const uint32_t*)(begin + 0xff000 + 8 * i);

      uint8_t vendor = endian_be32(p[0]) >> 24;

      if (!vendor)
        continue;

      uint16_t device = (endian_be32(p[0]) >> 12) & 0xfff;
      uint8_t version = (endian_be32(p[0]) >> 5) & 0x1f; 

#ifdef CONFIG_DEVICE_IRQ
      bool_t is_icu = (vendor == 0x01 && device == 0x00d);
      struct device_s *d = device_alloc(5 + (is_icu ? CONFIG_CPU_MAXCOUNT : 0));
#else
      struct device_s *d = device_alloc(5);
#endif

      if (!d)
        continue;

#ifdef CONFIG_GAISLER_DEVICE_IDS
      if (vendor < GAISLER_VENDOR_count &&
          gaisler_vendors_longnames[vendor])
        device_res_add_vendorid(d, vendor, strdup(gaisler_vendors_longnames[vendor]));
      else
#endif
        device_res_add_vendorid(d, vendor, NULL);

#ifdef CONFIG_GAISLER_DEVICE_IDS
      if (vendor == GAISLER_VENDOR_GAISLER &&
          device < GAISLER_DEVICE_count &&
          gaisler_devices_longnames[device])
        device_res_add_productid(d, device, strdup(gaisler_devices_longnames[device]));
      else
#endif
        device_res_add_productid(d, device, NULL);

      device_res_add_revision(d, version, 0);

#ifdef CONFIG_DEVICE_IRQ
      uint8_t irq = endian_be32(p[0]) & 0x1f;

      if (irq)
        device_res_add_irq(d, 0, irq - 1, 0, NULL);

      /* check for interrupt controller device */
      if (is_icu && gaisler_icu == NULL)
        {
          struct apbctrl_scan_cpu_irq_ctx_s ctx = { d, 0 };
          // keep track of gaisler single irq controller
          gaisler_icu = d;
          // add irq links from irqmp to cpu
          device_tree_walk(NULL, &apbctrl_scan_cpu_irq, &ctx);

          if (ctx.count == 0)
            printk("apbctrl: no processor found to link to irqmp device.");
        }
#endif

      uint32_t start = begin + (((endian_be32(p[1]) >> 20) & 0xfff) << 8);
      uint32_t mask = ~(((endian_be32(p[1]) >> 4) & 0xfff) << 8) & 0xfffff;

      if (mask & (mask+1))
        printk("apbctrl: %p device address mask with non contiguous range is not supported\n", d);
      else
        {
          switch (endian_be32(p[1]) & 0xf)
            {
            case 1:
              device_res_add_mem(d, start, start + mask + 1);
              break;
            default:
              ;
            }
        }

      d->enum_dev = dev;
      device_attach(d, dev);
      device_shrink(d);
    }
}

DEV_CLEANUP(apbctrl_cleanup);
DEV_INIT(apbctrl_init);

static const struct devenum_ident_s	gaisler_apbctrl_ids[] =
{
  DEVENUM_GAISLER_ENTRY(0x1, 0x006),
  { 0 }
};

static const struct driver_enum_s apbctrl_enum_drv =
{
  .class_	= DRIVER_CLASS_ENUM,
  .f_match_driver = apbctrl_match_driver,
#ifdef CONFIG_DEVICE_IRQ
  .f_get_default_icu = apbctrl_get_default_icu,
#endif
};

const struct driver_s	apbctrl_drv =
{
  .desc         = "Gaisler APB bus controller",
  .id_table	= gaisler_apbctrl_ids,
  .f_init	= apbctrl_init,
  .f_cleanup	= apbctrl_cleanup,
  .classes	= { &apbctrl_enum_drv, 0 }
};

REGISTER_DRIVER(apbctrl_drv);

DEV_INIT(apbctrl_init)
{
  dev->status = DEVICE_DRIVER_INIT_FAILED;
  dev->drv = &apbctrl_drv;

  uintptr_t begin, end;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &begin, &end))
    return -EINVAL;

  if (end - begin < 0x100000) // range needed for pnp data
    return -EINVAL;

  apbctrl_scan(dev, begin);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

  return -1;
}

DEV_CLEANUP(apbctrl_cleanup)
{
}

