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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <stdlib.h>
#include <limits.h>
#include <string.h>

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <inttypes.h>

struct termui_optctx_dev_opts
{
  struct device_s *dev;
};

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_device_parse)
{
  struct dev_console_opt_device_s *optd = (void*)opt;

  struct device_s **devp = (void*)((uint8_t*)ctx + optd->offset);

  if (device_get_by_path(devp, argv[0], optd->filter))
    return -ECANCELED;
  return 0;    
}

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_accessor_parse)
{
  struct dev_console_opt_accessor_s *optd = (void*)opt;

  void *accessor = (void*)((uint8_t*)ctx + optd->offset);

  if (device_get_accessor_by_path(accessor, NULL, argv[0], optd->cl))
    return -ECANCELED;
  return 0;
}

#ifdef CONFIG_LIBTERMUI_CON_COMPLETION

static void dev_console_opt_comp(struct termui_con_complete_ctx_s *cctx,
                                 enum driver_class_e cl, device_filter_t *filter)
{
  while (cctx->start < cctx->end && *cctx->start == '/')
    cctx->start++;
  char *s = cctx->start;

#ifdef CONFIG_DEVICE_TREE
  struct device_node_s *root = device_tree_root();

  while (s < cctx->end)
    {
    next:      
      if (*s == '/' || *s == '[')
        {
          size_t len = s - cctx->start;
          if (len)
            {
              DEVICE_NODE_FOREACH(root, node, {
                  if (!node->name || (node->flags & DEVICE_FLAG_IGNORE))
                    continue;
                  if (!strncmp(node->name, cctx->start, len) &&
                      !node->name[len])
                    {
                      root = node;
                      const char *b;
                      if (device_resolve_alias(&root, 8, &b))
                        return;
                      while (*s && *s != '/')
                        s++;
                      if (*s != '/')
                        return;
                      s++;
                      cctx->start = s;
                      goto next;
                    }
                });
              return;
            }
        }
      s++;
    }
#endif

  DEVICE_NODE_FOREACH(root, node, {
      struct device_node_s *n = node;
      uint_fast8_t i;
      if (!n->name || (n->flags & DEVICE_FLAG_IGNORE))
        continue;
      if (termui_con_comp_match(cctx, n->name, NULL, 0) <= 0)
        continue;
      if (filter && !filter(n))
        continue;
#ifdef CONFIG_DEVICE_TREE
      const char *b;
      if (device_resolve_alias(&n, 8, &b))
        continue;
#endif
      struct device_s *dev = device_from_node(n);
      if (dev == NULL)
        continue;
#ifdef CONFIG_DEVICE_TREE
      if (!device_list_isempty(&n->children))
        {
          cctx->suffix = '/';
        }
      else
#endif
        if (cl != DRIVER_CLASS_NONE)
          {
            const struct driver_class_s *c;
            if (dev->drv == NULL)
              continue;
            for (i = 0; (c = dev->drv->classes[i]) != NULL; i++)
              if (c->class_ == cl)
                break;
            if (c == NULL)
              continue;
          }
      if (!termui_con_comp_append(cctx, node->name))
        return;
  });
}

TERMUI_CON_ARGS_COLLECT_PROTOTYPE(dev_console_opt_accessor_comp)
{
  const struct dev_console_opt_accessor_s *optd = (void*)entry;

  dev_console_opt_comp(cctx, optd->cl, NULL);
  return NULL;
}

TERMUI_CON_ARGS_COLLECT_PROTOTYPE(dev_console_opt_device_comp)
{
  const struct dev_console_opt_device_s *optd = (void*)entry;

  dev_console_opt_comp(cctx, DRIVER_CLASS_NONE, optd->filter);
  return NULL;
}

TERMUI_CON_ARGS_COLLECT_PROTOTYPE(dev_console_device_comp)
{
  dev_console_opt_comp(cctx, DRIVER_CLASS_NONE, NULL);
  return NULL;
}
#endif

struct dev_console_fract_s
{
    uint64_t num;
    uint64_t denom;
};

static error_t dev_console_parse_fract(const char                 *arg,
                                       struct dev_console_fract_s *fract)
{
  char *ptr;

  fract->num = strtoll(arg, &ptr, 10);
  if (fract->num == LONG_MAX || fract->num == LONG_MIN || arg == ptr)
    return -EINVAL;

  if (*ptr == '\0')
    fract->denom = 1;
  else if ((*ptr == '/') || (*ptr == '.'))
    fract->denom = strtoll(ptr+1, NULL, 10);
  else
    return -EINVAL;

  if (fract->denom == LONG_MAX || fract->denom == LONG_MIN)
    return -EINVAL;

  if (*ptr == '.')
    {
      uint32_t div = 1;
      size_t   divlen = strlen(ptr+1);

      for (uint32_t i = 0; i < divlen; i++)
        div *= 10;

      fract->num   = fract->num * div + fract->denom;
      fract->denom = div;
    }

  return 0;
}

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_freq_parse)
{
  struct dev_console_opt_freq_s *optf = (void*)opt;
  struct dev_freq_s             *freq = (void*)((uint8_t*)ctx + optf->offset);

  struct dev_console_fract_s fract;
  if (dev_console_parse_fract(argv[0], &fract))
    return -ECANCELED;

  /* check coersion consistency. */
  if (fract.num & ~((1ULL << CONFIG_DEVICE_CLOCK_OSCN_WIDTH) - 1))
    return -ECANCELED;

  if (fract.denom & ~((1ULL << (64-CONFIG_DEVICE_CLOCK_OSCN_WIDTH)) - 1))
    return -ECANCELED;

  freq->num   = fract.num;
  freq->denom = fract.denom;
  return 0;
}

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_freq_ratio_parse)
{
  struct dev_console_opt_freq_s *optf  = (void*)opt;
  struct dev_freq_ratio_s       *ratio = (void*)((uint8_t*)ctx + optf->offset);

  struct dev_console_fract_s fract;
  if (dev_console_parse_fract(argv[0], &fract))
    return -ECANCELED;

  /* check coersion consistency. */
  if (fract.num & ~((1ULL << CONFIG_DEVICE_CLOCK_FRAC_WIDTH) - 1))
    return -ECANCELED;

  if (fract.denom & ~((1ULL << CONFIG_DEVICE_CLOCK_FRAC_WIDTH) - 1 ))
    return -ECANCELED;

  ratio->num   = fract.num;
  ratio->denom = fract.denom;
  return 0;
}

#ifdef CONFIG_DEVICE_TREE
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_alias)
{
  device_new_alias_to_path(NULL, argv[0], argv[1]);
  return 0;
}
#endif

/*************************************************** device tree dump */

#include <device/class/clock.h>

static void
dev_shell_dump_device(struct termui_console_s *con, struct device_s *dev, uint_fast8_t indent)
{
  uint_fast8_t i;

  termui_con_printf(con, "\n");
  for (i = 0; i < indent; i++)
    termui_con_printf(con, "  ");
  termui_con_printf(con, "Device %p `%s'", dev, dev->node.name);
  if (dev->node.flags & DEVICE_FLAG_IGNORE)
    termui_con_printf(con, " (ignored)");
  termui_con_printf(con, "\n");

  for (i = 0; i < indent; i++)
    termui_con_printf(con, "  ");
  termui_con_printf(con, "  status: ");
  termui_con_print_enum(con, device_status_e, dev->status);
  termui_con_printf(con, ", use: %i\n", dev->ref_count);

  if (dev->drv)
    {
      const struct driver_class_s *c;
      for (i = 0; i < indent + 1; i++)
        termui_con_printf(con, "  ");
      termui_con_printf(con, "Driver: %p `%s'\n", dev->drv, dev->drv->desc);
      for (i = 0; i < indent + 2; i++)
        termui_con_printf(con, "  ");
      termui_con_printf(con, "Classes: ");
      for (i = 0; (c = dev->drv->classes[i]); i++)
        {
          if (i > 0)
            termui_con_printf(con, ", ");
          termui_con_print_enum(con, driver_class_e, c->class_);
        }
      termui_con_printf(con, "\n");
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
        termui_con_printf(con, "  ");
      switch (type)
        {
        case DEV_RES_MEM:
          termui_con_printf(con, "  Memory range %i from %p to %p\n", c, r->u.mem.start, r->u.mem.end);
          break;
        case DEV_RES_IO:
          termui_con_printf(con, "  I/O range %i from %p to %p\n", c, r->u.io.start, r->u.io.end);
          break;
#ifdef CONFIG_DEVICE_IRQ
        case DEV_RES_IRQ: {
          termui_con_printf(con, "  IRQ %u connected to input %u:%u of controller `%s'\n",
                 r->u.irq.dev_out_id, r->u.irq.icu_in_id, r->u.irq.irq_id, r->u.irq.icu);
          break;
        }
#endif
#ifdef CONFIG_DEVICE_GPIO
        case DEV_RES_GPIO: {
          termui_con_printf(con, "  Pin `%s' connected to GPIO line %u (%u bit wide)\n",
                 r->u.gpio.label, r->u.gpio.id, r->u.gpio.width);
          break;
        };
#endif
#ifdef CONFIG_DEVICE_IOMUX
        case DEV_RES_IOMUX: {
          termui_con_printf(con, "  IO `%s' muxing: demux %u, id %u, mux %u, config %x\n",
                 r->u.iomux.label, r->u.iomux.demux,
                 r->u.iomux.io_id, r->u.iomux.mux, r->u.iomux.config);
          break;          
        }
#endif
#ifdef CONFIG_DEVICE_CLOCK
        case DEV_RES_CLOCK_RTE: {
          struct dev_clock_node_info_s info;
          const char *nname = "unknown";
          const char *pname = "unknown";
          if (!dev_clock_node_info(dev, r->u.clock_rte.node, DEV_CLOCK_INFO_NAME, &info))
            nname = info.name;
          if (!dev_clock_node_info(dev, r->u.clock_rte.parent, DEV_CLOCK_INFO_NAME, &info))
            pname = info.name;
          termui_con_printf(con, "  Clock route: node %u `%s': parent %u `%s', scale %"PRIu64"/%"PRIu64", config mask 0x%x\n",
                 r->u.clock_rte.node, nname, r->u.clock_rte.parent, pname,
                 (uint64_t)r->u.clock_rte.num, (uint64_t)r->u.clock_rte.denom,
                 r->u.clock_rte.config
          );
          break;
        }

        case DEV_RES_CLOCK_OSC: {
          struct dev_clock_node_info_s info;
          const char *nname = "unknown";
          if (!dev_clock_node_info(dev, r->u.clock_rte.node, DEV_CLOCK_INFO_NAME, &info))
            nname = info.name;
          uint64_t integral  = r->u.clock_osc.num / r->u.clock_osc.denom;
          uint32_t frac      = 1000 * (r->u.clock_osc.num % r->u.clock_osc.denom) /
                                 r->u.clock_osc.denom;
          termui_con_printf(con, "  Clock oscillator: node %"PRIuFAST8" `%s' @ %"PRIu64".%03"PRIu32" Hz, config mask 0x%x\n",
                 (uint_fast8_t)r->u.clock_osc.node, nname, (uint64_t)integral, (uint32_t)frac,
                 r->u.clock_osc.config
          );
          break;
        }

        case DEV_RES_CLOCK_SRC: {
          termui_con_printf(con, 
            "  Clock source `%s': src %u, sink %u\n",
            r->u.clock_src.src, r->u.clock_src.src_ep, r->u.clock_src.sink_ep
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
          termui_con_printf(con, 
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
          termui_con_printf(con, "  Numerical identifier %x %x\n", r->u.id.major, r->u.id.minor);
          break;
        case DEV_RES_VENDOR:
          termui_con_printf(con, "  Vendor ID 0x%04x `%s'\n", r->u.vendor.id, r->u.vendor.name);
          break;
        case DEV_RES_PRODUCT:
          termui_con_printf(con, "  Product ID 0x%04x `%s'\n", r->u.product.id, r->u.product.name);
          break;
        case DEV_RES_REVISION:
          termui_con_printf(con, "  Revision %u.%u\n", r->u.revision.major, r->u.revision.minor);
          break;
        case DEV_RES_FREQ: {
          uint64_t integral  = r->u.freq.num / r->u.freq.denom;
          uint32_t frac      = 1000 * (r->u.freq.num % r->u.freq.denom) /
                                 r->u.freq.denom;
          termui_con_printf(con, "  Frequency %"PRIu64".%03"PRIu32" Hz\n", (uint64_t)integral, (uint32_t)frac);
          break;
        }
        case DEV_RES_STR_PARAM:
          termui_con_printf(con, "  String parameter `%s' = `%s'\n", r->u.str_param.name, r->u.str_param.value);
          break;
        case DEV_RES_UINT_PARAM:
          termui_con_printf(con, "  Integer parameter `%s' = %u (0x%x)\n", r->u.uint_param.name,
                 r->u.uint_param.value, r->u.uint_param.value);
          break;
        case DEV_RES_DEV_PARAM:
          termui_con_printf(con, "  Device parameter `%s' = `%s'\n", r->u.dev_param.name, r->u.dev_param.dev);
          break;
        case DEV_RES_UINT_ARRAY_PARAM: {
          uintptr_t i;
          termui_con_printf(con, "  Array parameter `%s' = [", r->u.uint_array_param.name);
          for (i = 1; i <= r->u.uint_array_param.value[0]; i++)
            termui_con_printf(con, " 0x%x", r->u.uint_array_param.value[i]);
          termui_con_printf(con, " ]\n");
          break;
          }
        default:
          termui_con_printf(con, "  %i: unknown resource type %i\n", _i, r->type);
        case DEV_RES_UNUSED:
          ;
        }
    });
}

#ifdef CONFIG_DEVICE_TREE
static void
dev_shell_dump_alias(struct termui_console_s *con, struct device_alias_s *alias, uint_fast8_t indent)
{
  uint_fast8_t i;
  termui_con_printf(con, "\n");
  for (i = 0; i < indent; i++)
    termui_con_printf(con, "  ");
  termui_con_printf(con, "Device alias %p `%s'\n", alias, alias->node.name);
  for (i = 0; i < indent; i++)
    termui_con_printf(con, "  ");
  termui_con_printf(con, "  target: %s\n", alias->path);
}
#endif

static void
dev_shell_dump_node(struct termui_console_s *con, struct device_node_s *root, uint_fast8_t i)
{
  if (root->flags & DEVICE_FLAG_DEVICE)
    dev_shell_dump_device(con, (struct device_s*)root, i);
#ifdef CONFIG_DEVICE_TREE
  else if (root->flags & DEVICE_FLAG_ALIAS)
    dev_shell_dump_alias(con, (struct device_alias_s*)root, i);
#endif
  else
    termui_con_printf(con, "Unknows device node %p `%s'\n", root, root->name);

#ifdef CONFIG_DEVICE_TREE
  DEVICE_NODE_FOREACH(root, node, {
    dev_shell_dump_node(con, node, i+1);
  });
#endif
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_tree)
{
#ifdef CONFIG_DEVICE_TREE
  struct device_node_s *root = device_tree_root();
  dev_shell_dump_node(con, root, 0);
#else
  DEVICE_NODE_FOREACH(, node, {
    dev_shell_dump_node(con, node, 0);
  });
#endif
  return 0;
}

/*************************************************** device command groups */

extern TERMUI_CON_GROUP_DECL(dev_shell_clock_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_timer_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_mem_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_pwm_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_crypto_group);

static TERMUI_CON_GROUP_DECL(dev_shell_subgroup) =
{
  TERMUI_CON_ENTRY(dev_shell_tree, "tree")
#ifdef CONFIG_DEVICE_TREE
  TERMUI_CON_ENTRY(dev_shell_alias, "alias",
                   TERMUI_CON_COMPLETE(dev_console_device_comp, NULL)
		   TERMUI_CON_ARGS(2, 2)
                   )
#endif
#ifdef CONFIG_DEVICE_CLOCK
  TERMUI_CON_GROUP_ENTRY(dev_shell_clock_group, "clock")
#endif
#ifdef CONFIG_DEVICE_TIMER
  TERMUI_CON_GROUP_ENTRY(dev_shell_timer_group, "timer")
#endif
#ifdef CONFIG_DEVICE_MEM
  TERMUI_CON_GROUP_ENTRY(dev_shell_mem_group, "mem")
#endif
#ifdef CONFIG_DEVICE_PWM
  TERMUI_CON_GROUP_ENTRY(dev_shell_pwm_group, "pwm")
#endif
#ifdef CONFIG_DEVICE_CRYPTO
  TERMUI_CON_GROUP_ENTRY(dev_shell_crypto_group, "crypto")
#endif
  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_group) =
{
  TERMUI_CON_GROUP_ENTRY(dev_shell_subgroup, "dev")
};

MUTEK_SHELL_GROUP_REGISTER(dev_shell_group);

