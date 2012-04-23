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

#include <mutek/printk.h>
#include <device/device.h>
#include <device/driver.h>

#ifdef CONFIG_DEVICE_TREE
struct device_s device_enum_root;
#endif

static void
device_dump_r(struct device_s *dev, uint_fast8_t indent)
{
  uint_fast8_t i, j;
  const char *status[] = { DEVICE_STATUS_NAMES };

  printk("\n");
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("Device %p `%s'\n", dev, dev->name);
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("  status: %s, use: %i\n", status[dev->status], dev->ref_count);

  if (dev->drv)
    {
      const struct driver_class_s *c;
      for (i = 0; i < indent + 1; i++)
        printk("  ");
      printk("Driver: %p `%s'\n", dev->drv, dev->drv->desc);
      for (i = 0; i < indent + 2; i++)
        printk("  ");
      printk("Classes: ");
      for (i = 0; (c = dev->drv->classes[i]); i++)
        {
          static const char *cnames[] = { DRIVER_CLASS_NAMES };
          printk("%u-%s ", c->class_, c->class_ <= DRIVER_CLASS_Sys_Last ? cnames[c->class_] : "Custom");
        }
      printk("\n");
    }

  uint_fast8_t count[DEV_RES_TYPES_COUNT] = { 0 };

  for (j = 0; j < dev->res_count; j++)
    {
      struct dev_resource_s *r = dev->res + j;

      uint16_t type = r->type;
      uint_fast8_t c = 0;

      if (type < DEV_RES_TYPES_COUNT)
        c = count[type]++;

      if (type == DEV_RES_UNUSED)
          continue;
      for (i = 0; i < indent; i++)
        printk("  ");
      switch (type)
        {
        case DEV_RES_MEM:
          printk("  Memory range %i from %p to %p\n", c, r->mem.start, r->mem.end);
          break;
        case DEV_RES_IO:
          printk("  I/O range %i from %p to %p\n", c, r->io.start, r->io.end);
          break;
#ifdef CONFIG_DEVICE_IRQ
        case DEV_RES_IRQ: {
          struct device_s *icu = r->irq.icu;
          printk("  IRQ output %i bound to input %i of controller %p `%s'\n",
                 r->irq.dev_out_id, r->irq.icu_in_id, icu, icu ? icu->name : "default");
          break;
        }
#endif
        case DEV_RES_ID:
          printk("  Numerical identifier %x %x\n", r->id.major, r->id.minor);
          break;
        case DEV_RES_VENDORID:
          printk("  Vendor ID 0x%04x `%s'\n", r->vendor.id, r->vendor.name);
          break;
        case DEV_RES_PRODUCTID:
          printk("  Product ID 0x%04x `%s'\n", r->product.id, r->product.name);
          break;
        case DEV_RES_REVISION:
          printk("  Revision %u.%u\n", r->revision.major, r->revision.minor);
          break;
        case DEV_RES_STR_PARAM:
          printk("  Custom parameter `%s' = `%s'\n", r->str_param.name, r->str_param.value);
          break;
        case DEV_RES_UINT_PARAM:
          printk("  Custom parameter `%s' = %x\n", r->uint_param.name, r->uint_param.value);
          break;
        case DEV_RES_UINT_ARRAY_PARAM: {
          uintptr_t i;
          printk("  Custom parameter `%s' = [", r->uint_array_param.name);
          for (i = 1; i <= r->uint_array_param.value[0]; i++)
            printk(" 0x%x", r->uint_array_param.value[i]);
          printk(" ]\n");
          break;
          }
        default:
          printk("  %i: unknown resource type %i\n", j, r->type);
        case DEV_RES_UNUSED:
          ;
        }
    }
}

void
device_dump(struct device_s *dev)
{
  device_dump_r(dev, 0);
}

#ifdef CONFIG_DEVICE_TREE

static void
device_dump_tree_r(struct device_s *root, uint_fast8_t i)
{
  device_dump_r(root, i);

  CONTAINER_FOREACH(device_list, CLIST, &root->children,
  {
    device_dump_tree_r(item, i+1);
  });
}

void
device_dump_tree(struct device_s *root)
{
  if (!root)
    root = &device_enum_root;

  device_dump_tree_r(&device_enum_root, 0);
}

#endif

