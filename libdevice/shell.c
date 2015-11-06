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

enum dev_opts_e
{
  DEV_OPT_DEV    = 0x01,
  DEV_OPT_NAME   = 0x02,
#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
  DEV_OPT_DRV    = 0x04,
#endif
  DEV_OPT_VERBOSE = 0x08,
  DEV_OPT_CLASS = 0x10,
};

struct termui_optctx_dev_opts
{
  struct device_s *dev;
  uint_fast8_t num;
  const char *name;
#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
  struct driver_s *drv;
#endif
  enum driver_class_e cl;
};

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_device_parse)
{
  struct dev_console_opt_device_s *optd = (void*)opt;

  struct device_s **devp = (void*)((uint8_t*)ctx + optd->dev_offset);
  uint_fast8_t *nump = (void*)((uint8_t*)ctx + optd->num_offset);

  if (device_get_by_path(devp, nump, NULL, argv[0], optd->filter))
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

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_driver_parse)
{
#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
  struct dev_console_opt_driver_s *optd = (void*)opt;
  const struct driver_registry_s *reg = driver_registry_table;

  const struct driver_s **drvp = (void*)((uint8_t*)ctx + optd->offset);
  const void *drv = (void*)strtoptr(argv[0], NULL, 16);

  for ( ; reg < driver_registry_table_end ; reg++ )
    {
      if (reg->driver != drv)
        continue;
      *drvp = drv;
      return 0;
    }
#endif
  return -ECANCELED;
}

#ifdef CONFIG_LIBTERMUI_CON_COMPLETION

static void dev_console_opt_comp(struct termui_con_complete_ctx_s *cctx,
                                 enum driver_class_e cl, device_filter_t *filter)
{
  while (cctx->start < cctx->end && *cctx->start == '/')
    cctx->start++;

#ifdef CONFIG_DEVICE_TREE
  char *s = cctx->start;
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
            uint_fast8_t num = 0;
            switch (device_last_number(dev, cl, &num))
              {
              case 0:
                cctx->suffix = '[';
              case -ENOTSUP:    /* no subdevice */
                break;
              default:          /* device not usable */
                continue;
              }
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
  struct termui_optctx_dev_opts *c = ctx;
  const char *name = "alias";

  if (used & DEV_OPT_NAME)
    name = c->name;
  else if (used & DEV_OPT_DEV)
    name = c->dev->node.name;

  if (used & DEV_OPT_DEV)
    device_new_alias_to_node(NULL, name, &c->dev->node);
  else if (argc == 1)
    device_new_alias_to_path(NULL, name, argv[0]);
  else
    return -EINVAL;

  return 0;
}

# ifdef CONFIG_DEVICE_DRIVER_CLEANUP
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_remove)
{
  struct termui_optctx_dev_opts *c = ctx;
  if (device_detach(c->dev))
    return -EINVAL;
  device_cleanup(c->dev);

  return 0;
}
# endif

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_new)
{
  struct termui_optctx_dev_opts *c = ctx;
  struct device_s *dev = device_alloc(8);

  if (dev == NULL)
    return -EINVAL;
  if (used & DEV_OPT_NAME)
    device_set_name(dev, c->name);

  device_attach(dev, c->dev);
  return 0;
}
#endif

/*************************************************** device tree dump */

#include <device/class/clock.h>

static void
dev_shell_dump_drv_class(struct termui_console_s *con, const struct driver_s *drv,
                         uint8_t pending_mask)
{
  const struct driver_class_s *c;
  uint_fast8_t i;

  for (i = 0; (c = drv->classes[i]); i++)
    {
      termui_con_printf(con, "%N%s, ", c->class_, driver_class_e,
                        pending_mask & 1 ? " (pending)" : "");
      pending_mask >>= 1;
    }
}

static void
dev_shell_dump_device(struct termui_console_s *con, struct device_s *dev,
                      uint_fast8_t indent, bool_t show_resources)
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
  termui_con_printf(con, "  status: %N, use: %u\n", dev->status, device_status_e, dev->ref_count);

  if (dev->drv)
    {
      for (i = 0; i < indent + 1; i++)
        termui_con_printf(con, "  ");
#if defined(CONFIG_DEVICE_DRIVER_DESC)
      termui_con_printf(con, "Driver: %p `%s'\n", dev->drv, dev->drv->desc);
#else
      termui_con_printf(con, "Driver: %p\n", dev->drv);
#endif
      for (i = 0; i < indent + 2; i++)
        termui_con_printf(con, "  ");
      termui_con_printf(con, "Classes: ");
      dev_shell_dump_drv_class(con, dev->drv, dev->status == DEVICE_DRIVER_INIT_PARTIAL ? ~dev->init_mask : 0);
      termui_con_printf(con, "\n");
    }

  if (!show_resources)
    return;

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
          termui_con_printf(con, "  IRQ source %u connected to sink %u of icu, irq id %u\n",
                 r->u.irq.src_id, r->u.irq.sink_id, r->u.irq.irq_id);
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
          const char *nname = "?";
          if (!dev_clock_node_info(dev, r->u.clock_osc.node, DEV_CLOCK_INFO_NAME, &info))
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
          static const char uart_parity[] = "NOE";
          termui_con_printf(con,
            "  UART: %d baud, %d-%c-%d"
              "%s%s\n",
            r->u.uart.baudrate,
            r->u.uart.data_bits,
            uart_parity[r->u.uart.parity],
            r->u.uart.stop_bits,
            (r->u.uart.flow_ctrl ? ", flow control" : ""),
            (r->u.uart.half_duplex ? ", half duplex" : "")
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
          termui_con_printf(con, "  Device parameter `%s' = `%s' (%N)\n",
                            r->u.dev_param.name, r->u.dev_param.dev, r->u.dev_param.class_, driver_class_e);
          break;
        case DEV_RES_UINT_ARRAY_PARAM: {
          uintptr_t i;
          termui_con_printf(con, "  Array parameter `%s' = [", r->u.uint_array_param.name);
          for (i = 0; i < r->u.uint_array_param.count; i++)
            termui_con_printf(con, " 0x%x", r->u.uint_array_param.array[i]);
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
dev_shell_dump_node(struct termui_console_s *con, struct device_node_s *root,
                    uint_fast8_t i, bool_t show_resources)
{
  if (root->flags & DEVICE_FLAG_DEVICE)
    dev_shell_dump_device(con, (struct device_s*)root, i, show_resources);
#ifdef CONFIG_DEVICE_TREE
  else if (root->flags & DEVICE_FLAG_ALIAS)
    dev_shell_dump_alias(con, (struct device_alias_s*)root, i);
#endif
  else
    termui_con_printf(con, "Unknows device node %p `%s'\n", root, root->name);

#ifdef CONFIG_DEVICE_TREE
  DEVICE_NODE_FOREACH(root, node, {
      dev_shell_dump_node(con, node, i+1, show_resources);
  });
#endif
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_tree)
{
  struct termui_optctx_dev_opts *c = ctx;
#ifdef CONFIG_DEVICE_TREE
  struct device_node_s *root = used & DEV_OPT_DEV ? &c->dev->node : device_tree_root();
  dev_shell_dump_node(con, root, 0, !!(used & DEV_OPT_VERBOSE));
#else
  if (used & DEV_OPT_DEV)
    {
      dev_shell_dump_node(con, &c->dev->node, 0, !!(used & DEV_OPT_VERBOSE));
    }
  else
    {
      DEVICE_NODE_FOREACH(, node, {
          dev_shell_dump_node(con, node, 0, !!(used & DEV_OPT_VERBOSE));
        });
    }
#endif
  return 0;
}

static error_t dev_shell_start_stop(struct termui_console_s *con, enum dev_opts_e used,
                                    struct termui_optctx_dev_opts *c, bool_t start)
{
  const struct driver_s *drv = c->dev->drv;

  if (!(used & DEV_OPT_CLASS))
    {
      /* guess device class */
      if (drv && drv->classes[0] && !drv->classes[1])
        c->cl = drv->classes[0]->class_;
      else
        {
          termui_con_printf(con, "Ambiguous device class, use --class\n");
          return -EINVAL;
        }
    }

  struct device_accessor_s acc;

  if (device_get_accessor(&acc, c->dev, c->cl, c->num))
    return -EINVAL;

  error_t err = start ? device_start(&acc) : device_stop(&acc);
  device_put_accessor(&acc);

  return err ? -EINVAL : 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_start)
{
  return dev_shell_start_stop(con, used, ctx, 1);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_stop)
{
  return dev_shell_start_stop(con, used, ctx, 0);
}

#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_driver_list)
{
  const struct driver_registry_s *reg = driver_registry_table;

  for ( ; reg < driver_registry_table_end ; reg++ )
    {
      const struct driver_s *d = reg->driver;
      size_t i;
      if (!d)
        continue;

#if defined(CONFIG_DEVICE_DRIVER_DESC)
      termui_con_printf(con, "\n  Driver %p `%s'", d, d->desc);
#else
      termui_con_printf(con, "\n  Driver %p", d);
#endif
      termui_con_printf(con, "\n    Classes: ");
      dev_shell_dump_drv_class(con, d, 0);
      termui_con_printf(con, "\n");

      if (!(used & DEV_OPT_VERBOSE))
        continue;

      for (i = 0; i < reg->id_count; ++i)
        {
          const struct dev_enum_ident_s *id = reg->id_table + i;

          termui_con_printf(con, "    Id: %N", id->type, dev_enum_type_e);
          switch (id->type)
            {
            case DEV_ENUM_TYPE_GENERIC:
              termui_con_printf(con, ", vendor %04x, device %04x, rev %u.%u\n",
                                id->generic.vendor, id->generic.device,
                                id->generic.rev_major,
                                id->generic.rev_minor);
              break;
            case DEV_ENUM_TYPE_FDTNAME:
              termui_con_printf(con, ", name `%s'\n", id->fdtname.name);
              break;
            default:
              termui_con_printf(con, ", %P\n", id, sizeof(*id));
              break;
            }
        }
    }
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_driver_bind)
{
  struct termui_optctx_dev_opts *c = ctx;
  return device_bind_driver(c->dev, c->drv) ? -EINVAL : 0;
}

# ifdef CONFIG_DEVICE_DRIVER_CLEANUP
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_driver_unbind)
{
  struct termui_optctx_dev_opts *c = ctx;
  device_release_driver(c->dev);
  return device_unbind_driver(c->dev) ? -EINVAL : 0;
}
# endif
#endif

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_driver_init)
{
  struct termui_optctx_dev_opts *c = ctx;
  return device_init_driver(c->dev) ? -EINVAL : 0;
}

#ifdef CONFIG_DEVICE_DRIVER_CLEANUP
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_driver_release)
{
  struct termui_optctx_dev_opts *c = ctx;
  return device_release_driver(c->dev) ? -EINVAL : 0;
}
#endif

static TERMUI_CON_OPT_DECL(dev_opts) =
{
  TERMUI_CON_OPT_DEV_DEVICE_ENTRY("-d", "--device", DEV_OPT_DEV,
                                  struct termui_optctx_dev_opts, dev, num, NULL,
                                  TERMUI_CON_OPT_CONSTRAINTS(DEV_OPT_DEV, 0)
                                  )

#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
  TERMUI_CON_OPT_DEV_DRIVER_ENTRY("-D", "--driver", DEV_OPT_DRV,
                                  struct termui_optctx_dev_opts, drv,
                                  TERMUI_CON_OPT_CONSTRAINTS(DEV_OPT_DRV, 0)
                                  )
#endif

  TERMUI_CON_OPT_CSTRING_ENTRY("-n", "--name", DEV_OPT_NAME,
                               struct termui_optctx_dev_opts, name, 1)

  TERMUI_CON_OPT_ENTRY("-v", "--verbose", DEV_OPT_VERBOSE,
                       TERMUI_CON_OPT_CONSTRAINTS(DEV_OPT_VERBOSE, 0)
                       )

  TERMUI_CON_OPT_ENUM_ENTRY("-c", "--class", DEV_OPT_CLASS, struct termui_optctx_dev_opts,
                            cl, driver_class_e,
                            TERMUI_CON_OPT_CONSTRAINTS(DEV_OPT_CLASS, 0))

  TERMUI_CON_LIST_END
};

static TERMUI_CON_GROUP_DECL(dev_driver_group) =
{
  TERMUI_CON_ENTRY(dev_shell_driver_init, "init",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV, 0, NULL)
                   )
#if defined(CONFIG_DEVICE_DRIVER_CLEANUP)
  TERMUI_CON_ENTRY(dev_shell_driver_release, "release",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV, 0, NULL)
                   )
# ifdef CONFIG_DEVICE_DRIVER_REGISTRY
  TERMUI_CON_ENTRY(dev_shell_driver_unbind, "unbind",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV, 0, NULL)
                   )
# endif
#endif
#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
  TERMUI_CON_ENTRY(dev_shell_driver_bind, "bind",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV | DEV_OPT_DRV, 0, NULL)
                   )
  TERMUI_CON_ENTRY(dev_shell_driver_list, "list",
                   TERMUI_CON_OPTS_CTX(dev_opts, 0, DEV_OPT_VERBOSE, NULL)
                   )
#endif

  TERMUI_CON_LIST_END
};

/*************************************************** device command groups */

extern TERMUI_CON_GROUP_DECL(dev_shell_clock_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_timer_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_mem_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_pwm_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_crypto_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_i2c_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_gpio_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_rfpacket_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_spi_ctrl_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_char_group);

static TERMUI_CON_GROUP_DECL(dev_shell_subgroup) =
{
  TERMUI_CON_ENTRY(dev_shell_tree, "tree",
                   TERMUI_CON_OPTS_CTX(dev_opts, 0, DEV_OPT_DEV | DEV_OPT_VERBOSE, NULL)
                   )
  TERMUI_CON_ENTRY(dev_shell_start, "start",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV, DEV_OPT_CLASS, NULL)
                   )
  TERMUI_CON_ENTRY(dev_shell_stop, "stop",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV, DEV_OPT_CLASS, NULL)
                   )
#ifdef CONFIG_DEVICE_TREE
# ifdef CONFIG_DEVICE_DRIVER_CLEANUP
  TERMUI_CON_ENTRY(dev_shell_remove, "remove",
                   TERMUI_CON_OPTS_CTX(dev_opts, DEV_OPT_DEV, 0, NULL)
                   )
# endif
  TERMUI_CON_ENTRY(dev_shell_new, "new",
                   TERMUI_CON_OPTS_CTX(dev_opts, 0, DEV_OPT_DEV | DEV_OPT_NAME, NULL)
                   )
  TERMUI_CON_ENTRY(dev_shell_alias, "alias",
                   TERMUI_CON_OPTS_CTX(dev_opts, 0, DEV_OPT_DEV | DEV_OPT_NAME, NULL)
                   TERMUI_CON_COMPLETE(dev_console_device_comp, NULL)
		   TERMUI_CON_ARGS(0, 1)
                   )
#endif
  TERMUI_CON_GROUP_ENTRY(dev_driver_group, "driver")
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
#ifdef CONFIG_DEVICE_I2C
  TERMUI_CON_GROUP_ENTRY(dev_shell_i2c_group, "i2c")
#endif
#ifdef CONFIG_DEVICE_GPIO
  TERMUI_CON_GROUP_ENTRY(dev_shell_gpio_group, "gpio")
#endif
#ifdef CONFIG_DEVICE_RFPACKET
  TERMUI_CON_GROUP_ENTRY(dev_shell_rfpacket_group, "rfpacket")
#endif
#ifdef CONFIG_DEVICE_SPI
  TERMUI_CON_GROUP_ENTRY(dev_shell_spi_ctrl_group, "spi")
#endif
#ifdef CONFIG_DEVICE_CHAR
  TERMUI_CON_GROUP_ENTRY(dev_shell_char_group, "char")
#endif
  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(dev_shell_subgroup, "dev")

