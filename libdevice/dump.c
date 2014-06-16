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
#include <device/resources.h>
#include <device/driver.h>

static void
device_dump_device(struct device_s *dev, uint_fast8_t indent)
{
  uint_fast8_t i;
  const char *status[] = { DEVICE_STATUS_NAMES };

  printk("\n");
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("Device %p `%s'", dev, dev->node.name);
  if (dev->node.flags & DEVICE_FLAG_IGNORE)
    printk(" (ignored)");
  printk("\n");

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
          if (c->class_ <= DRIVER_CLASS_Sys_Last)
            printk("%s, ", cnames[c->class_]);
          else
            printk("Custom(%u), ", c->class_);
         }
      printk("\n");
    }

  uint_fast8_t count[DEV_RES_TYPES_COUNT] = { 0 };

  DEVICE_RES_FOREACH(dev, r, {

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
          printk("  Memory range %i from %p to %p\n", c, r->u.mem.start, r->u.mem.end);
          break;
        case DEV_RES_IO:
          printk("  I/O range %i from %p to %p\n", c, r->u.io.start, r->u.io.end);
          break;
#ifdef CONFIG_DEVICE_IRQ
        case DEV_RES_IRQ: {
          printk("  IRQ %u connected to input %u:%u of controller `%s'\n",
                 r->u.irq.dev_out_id, r->u.irq.icu_in_id, r->u.irq.irq_id, r->u.irq.icu);
          break;
        }
#endif
#ifdef CONFIG_DEVICE_GPIO
        case DEV_RES_GPIO: {
          printk("  Pin `%s' connected to GPIO line %u (%u bit wide)\n",
                 r->u.gpio.label, r->u.gpio.id, r->u.gpio.width);
          break;
        };
#endif
#ifdef CONFIG_DEVICE_IOMUX
        case DEV_RES_IOMUX: {
          printk("  IO `%s' muxing: demux %u, id %u, mux %u, config %u\n",
                 r->u.iomux.label, r->u.iomux.demux,
                 r->u.iomux.io_id, r->u.iomux.mux, r->u.iomux.config);
          break;          
        }
#endif
#ifdef CONFIG_DEVICE_CLOCK
        case DEV_RES_CLOCK_RTE: {
          printk(
            "  Clock route: src %u -> sink %u, scale %u/%u, config %u\n",
            r->u.clock_rte.in, r->u.clock_rte.out, r->u.clock_rte.num,
            r->u.clock_rte.denum, r->u.clock_rte.cfg
          );
          break;
        }

        case DEV_RES_CLOCK_OSC: {
          uint32_t integral = r->u.clock_osc.integral;
          uint32_t frac = 0;

          if (r->u.clock_osc.num < r->u.clock_osc.denum)
            frac = 1000 * r->u.clock_osc.num / r->u.clock_osc.denum;
          else
            {
              integral += r->u.clock_osc.num / r->u.clock_osc.denum;
              frac      = 1000 * (r->u.clock_osc.num % r->u.clock_osc.denum) /
                r->u.clock_osc.denum;
            }

          printk(
            "  Clock oscillator: src %u, freq %u + %u/%u (%u.%03u) Hz\n",
            r->u.clock_osc.id,
            r->u.clock_osc.integral, r->u.clock_osc.num, r->u.clock_osc.denum,
            integral, frac
          );
          break;
        }

        case DEV_RES_CLOCK_SRC: {
          printk(
            "  Clock source `%s': src %u, sink %u\n",
            r->u.clock_src.src, r->u.clock_src.in, r->u.clock_src.out
          );
          break;
        }
#endif
#ifdef CONFIG_DEVICE_UART
        case DEV_RES_UART: {
          static const char * uart_baudrates[] = {
            "110", "300", "600", "1200", "2400", "4800", "9600", "14400",
            "19200", "28800", "38400", "56000", "57600", "115200"
          };
          static const char * uart_data_bits[] = {
            "6 bits", "7 bits", "8 bits", "9 bits"
          };
          static const char * uart_stop_bits[] = {
            "1 bit", "2 bits"
          };
          static const char * uart_parity[] = {
            "none", "odd", "even"
          };
          printk(
            "  UART: baudrate %s, data %s, stop %s, parity %s,"
              " flow ctrl %s, half dup %s\n",
            uart_baudrates[r->u.uart.baudrate],
            uart_data_bits[r->u.uart.data_bits],
            uart_stop_bits[r->u.uart.stop_bits],
            uart_parity[r->u.uart.parity],
            (r->u.uart.flow_ctrl == 0 ? "false" : "true"),
            (r->u.uart.half_duplex == 0 ? "false" : "true")
          );
          break;
        }
#endif
        case DEV_RES_ID:
          printk("  Numerical identifier %x %x\n", r->u.id.major, r->u.id.minor);
          break;
        case DEV_RES_VENDOR:
          printk("  Vendor ID 0x%04x `%s'\n", r->u.vendor.id, r->u.vendor.name);
          break;
        case DEV_RES_PRODUCT:
          printk("  Product ID 0x%04x `%s'\n", r->u.product.id, r->u.product.name);
          break;
        case DEV_RES_REVISION:
          printk("  Revision %u.%u\n", r->u.revision.major, r->u.revision.minor);
          break;
        case DEV_RES_FREQ:
          printk("  Frequency %u.%07u\n", (uint32_t)(r->u.freq.f40_24 >> 24),
                 (uint32_t)((r->u.freq.f40_24 & 0xffffffULL) * 59604644 / 100000000));
          break;
        case DEV_RES_STR_PARAM:
          printk("  String parameter `%s' = `%s'\n", r->u.str_param.name, r->u.str_param.value);
          break;
        case DEV_RES_UINT_PARAM:
          printk("  Integer parameter `%s' = %u (0x%x)\n", r->u.uint_param.name,
                 r->u.uint_param.value, r->u.uint_param.value);
          break;
        case DEV_RES_DEV_PARAM:
          printk("  Device parameter `%s' = `%s'\n", r->u.dev_param.name, r->u.dev_param.dev);
          break;
        case DEV_RES_UINT_ARRAY_PARAM: {
          uintptr_t i;
          printk("  Array parameter `%s' = [", r->u.uint_array_param.name);
          for (i = 1; i <= r->u.uint_array_param.value[0]; i++)
            printk(" 0x%x", r->u.uint_array_param.value[i]);
          printk(" ]\n");
          break;
          }
        default:
          printk("  %i: unknown resource type %i\n", _i, r->type);
        case DEV_RES_UNUSED:
          ;
        }
    });
}

void
device_dump(struct device_s *dev)
{
  device_dump_device(dev, 0);
}

#ifdef CONFIG_DEVICE_TREE
static void
device_dump_alias(struct device_alias_s *alias, uint_fast8_t indent)
{
  uint_fast8_t i;
  printk("\n");
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("Device alias %p `%s'\n", alias, alias->node.name);
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("  target: %s\n", alias->path);
}
#endif

static void
device_dump_node(struct device_node_s *root, uint_fast8_t i)
{
  if (root->flags & DEVICE_FLAG_DEVICE)
    device_dump_device((struct device_s*)root, i);
#ifdef CONFIG_DEVICE_TREE
  else if (root->flags & DEVICE_FLAG_ALIAS)
    device_dump_alias((struct device_alias_s*)root, i);
#endif
  else
    printk("Unknows device node %p `%s'\n", root, root->name);

#ifdef CONFIG_DEVICE_TREE
  DEVICE_NODE_FOREACH(root, node, {
    device_dump_node(node, i+1);
  });
#endif
}

void
device_dump_tree(struct device_node_s *root)
{
#ifdef CONFIG_DEVICE_TREE
  if (!root)
    root = device_tree_root();
  device_dump_node(root, 0);
#else
  DEVICE_NODE_FOREACH(, node, {
    device_dump_node(node, 0);
  });
#endif
}

