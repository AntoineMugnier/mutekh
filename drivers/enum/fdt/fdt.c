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

    Copyright (c) 2009, Nicolas Pouillon <nipo@ssji.net>
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012
*/


#include <hexo/types.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <hexo/local.h>

#include <device/class/enum.h>
#include <device/class/icu.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <string.h>

#include <fdt/reader.h>

enum enum_fdt_section_e
{
  FDT_SECTION_NONE,
  FDT_SECTION_CPUS,
  FDT_SECTION_DEVICE,
  FDT_SECTION_CHOSEN,
  FDT_SECTION_ALIAS,
};

struct enum_fdt_stack_entry_s
{
  struct device_s *dev;
  uint32_t icu_phandle;
  uint8_t addr_cells;
  uint8_t size_cells;
#ifdef CONFIG_DEVICE_IRQ
  uint8_t interrupt_cells;
#endif
  enum enum_fdt_section_e section;
};

#if CONFIG_DRIVER_ENUM_FDT_MAX_DEPTH < 2
# error CONFIG_DRIVER_ENUM_FDT_MAX_DEPTH must be at least 2
#endif

#if CONFIG_DRIVER_ENUM_FDT_MAX_RESOURCES < 1
# error CONFIG_DRIVER_ENUM_FDT_MAX_RESOURCES must be at least 1
#endif

struct enum_fdt_parse_ctx_s
{
  struct device_s *dev; // enum fdt dev
  struct enum_fdt_stack_entry_s stack[CONFIG_DRIVER_ENUM_FDT_MAX_DEPTH];
  int_fast8_t stack_top;
};

static const char * fdt_name(const char *path)
{
  const char *n = path;

  while (*path)
    {
      if (*path == '/')
        n = path + 1;
      path++;
    }

  return n;
}

static FDT_ON_NODE_ENTRY_FUNC(enum_fdt_node_entry)
{
  struct enum_fdt_parse_ctx_s *ctx = priv;
  const char *name = fdt_name(path);

  ctx->stack_top++;

  if (ctx->stack_top == 0)
    return 1;

  if (ctx->stack_top >= CONFIG_DRIVER_ENUM_FDT_MAX_DEPTH)
    return 0;

  struct enum_fdt_stack_entry_s *e = ctx->stack + ctx->stack_top;
  struct enum_fdt_stack_entry_s *p = ctx->stack + ctx->stack_top - 1;

  *e = *p;

  if (p->dev == NULL)
    return 0;

  switch (p->section)
    {
    case FDT_SECTION_NONE:
      if (!strcmp(name, "cpus"))
        {
          e->section = FDT_SECTION_CPUS;
          return 1;
        }
      else if (!strcmp(name, "chosen"))
        {
          e->section = FDT_SECTION_CHOSEN;
          return 1;
        }
      else if (!strcmp(name, "aliases"))
        {
          e->section = FDT_SECTION_ALIAS;
          return 1;
        }

    case FDT_SECTION_CPUS:
    case FDT_SECTION_DEVICE:
      if (strchr(name, '@'))
        {
          struct device_s *d = device_alloc(CONFIG_DRIVER_ENUM_FDT_MAX_RESOURCES);

          d->enum_dev = ctx->dev;

          if (d)
            {
              device_set_name(d, name);
              device_attach(d, p->dev);

              d->enum_pv = (void*)-1;
              if (p->section == FDT_SECTION_CPUS)
                d->node.flags |= DEVICE_FLAG_CPU;
            }

          e->dev = d;
          if (e->section == FDT_SECTION_NONE)
            e->section = FDT_SECTION_DEVICE;
          return 1;
        }

    default:
      printk("enum-fdt: ignored node `%s'\n", path);
    }

  return 0;
}

static FDT_ON_NODE_LEAVE_FUNC(enum_fdt_node_leave)
{
  struct enum_fdt_parse_ctx_s *ctx = priv;

  struct enum_fdt_stack_entry_s *e = ctx->stack + ctx->stack_top;

  if (e->dev != ctx->dev)
    device_shrink(e->dev);

  ctx->stack_top--;
}

static struct device_s *enum_fdt_get_phandle(struct device_s *dev, uint32_t phandle)
{
  CONTAINER_FOREACH_NOLOCK(device_list, CLIST, &dev->node.children, {

      if (!(item->flags & DEVICE_FLAG_DEVICE))
        CONTAINER_FOREACH_CONTINUE;

      struct device_s *d = (struct device_s*)item;

      if ((uint32_t)d->enum_pv == phandle)
        return d;

      struct device_s *r = enum_fdt_get_phandle(d, phandle);

      if (r)
        return r;
    });
  return NULL;
}

static FDT_ON_NODE_PROP_FUNC(enum_fdt_node_prop)
{
  struct enum_fdt_parse_ctx_s *ctx = priv;

  if (ctx->stack_top >= CONFIG_DRIVER_ENUM_FDT_MAX_DEPTH)
    return;

  struct enum_fdt_stack_entry_s *e = ctx->stack + ctx->stack_top;
  const uint8_t *data8 = data;

  /* misc sections */

  switch (e->section)
    {
    case FDT_SECTION_ALIAS:
      if (datalen && !data8[datalen - 1])
        {
          char buf[128];
          ssize_t l = device_get_path(NULL, buf, sizeof(buf), &ctx->dev->node, 0);
          if (l >= 0 && l + 1 + datalen < sizeof(buf))
            {
              buf[l] = '/';
              memcpy(buf + l + 1, data8, datalen);
              buf[l + 1 + datalen] = 0;
              device_new_alias_to_path(NULL, name, buf);
            }
          else
            {
              printk("enum-fdt: ignored entry in `aliases' section `%s : %P'\n", name, data, datalen);
            }
        }
      return;

    case FDT_SECTION_CHOSEN:
      printk("enum-fdt: ignored entry in `chosen' section `%s : %P'\n", name, data, datalen);
      return;

    default:
      break;
    }

  /* cpu and device sections */

  switch (name[0])
    {
    case '#':
      if (!strcmp(name + 1, "address-cells") && datalen == 4)
        {
          e->addr_cells = endian_be32(*(const uint32_t*)data);
          return;
        }
      else if (!strcmp(name + 1, "size-cells") && datalen == 4)
        {
          e->size_cells = endian_be32(*(const uint32_t*)data);
          return;
        }
#ifdef CONFIG_DEVICE_IRQ
      else if (!strcmp(name + 1, "interrupt-cells") && datalen == 4)
        {
          e->interrupt_cells = endian_be32(*(const uint32_t*)data);
          return;
        }
#endif
      break;

    case 'r': {
      uint32_t elen = (e->addr_cells + e->size_cells) * 4;

      if (!strcmp(name + 1, "eg") && datalen >= elen && e->dev != ctx->dev)
        switch (e->section)
          {
            uintptr_t a, b;
          case FDT_SECTION_CPUS:
            a = 0;
            fdt_parse_cell(data, e->addr_cells, &a);
            if (device_res_add_id(e->dev, a, 0))
              goto res_err;
            return;
          case FDT_SECTION_DEVICE:
            while (datalen >= elen)
              {
                a = b = 0;
                fdt_parse_cell(fdt_parse_cell(data8, e->addr_cells, &a), e->size_cells, &b);
                if (device_res_add_mem(e->dev, a, a + b))
                  goto res_err;
                datalen -= elen;
                data8 += elen;
              }
            return;
          default:
            break;
          }
      break;
    }

    case 'f': {
      if (!strcmp(name + 1, "requency") && datalen >= 4 && e->dev != ctx->dev)
        switch (e->section)
          {
            uintptr_t f;
          case FDT_SECTION_DEVICE:
          case FDT_SECTION_CPUS:
            while (datalen >= 4)
              {
                f = 0;
                fdt_parse_cell(data8, e->addr_cells, &f);
                struct dev_freq_s freq = {
                  .num = f, .denom = 1
                };
                if (device_res_add_freq(e->dev, &freq))
                  goto res_err;
                datalen -= 4;
                data8 += 4;
              }
            return;
          default:
            break;
          }
      break;
    }

    case 'c':
      if (!strcmp(name + 1, "ompatible") && datalen && e->dev != ctx->dev)
        switch (e->section)
          {
          case FDT_SECTION_CPUS:
          case FDT_SECTION_DEVICE: {
            if (((char*)data)[datalen])
              goto res_err;
            if (device_res_add_product(e->dev, 0, data))
              goto res_err;
            return;
          }
          case FDT_SECTION_NONE:
            return;
          default:
            break;
          }
      break;

    case 'l':
      if (!strcmp(name + 1, "inux,phandle") && datalen == 4 && e->dev != ctx->dev)
        {
          e->dev->enum_pv = (void*)endian_be32(*(const uint32_t*)data);
          return;
        }
      break;

    case 'p':
      if (e->dev != ctx->dev)
        {
          if (!strncmp(name + 1, "aram-str-", 9) && !((char*)data)[datalen])
            {
              if (device_res_add_str_param(e->dev, name + 10, data))
                goto res_err;
              return;
            }
          else if (!strncmp(name + 1, "aram-int-", 9) && datalen == 4)
            {
              if (device_res_add_uint_param(e->dev, name + 10, endian_be32(*(const uint32_t*)data)))
                goto res_err;
              return;
            }
          else if (!strncmp(name + 1, "aram-device-path-", 17) && !((char*)data)[datalen])
            {
              if (device_res_add_dev_param(e->dev, name + 18, data))
                goto res_err;
              return;
            }
          else if (!strncmp(name + 1, "aram-device-", 12) && datalen == 4)
            {
              struct dev_resource_s *r;
              if (device_res_alloc_str(e->dev, DEV_RES_DEV_PARAM, NULL, name + 13, &r))
                goto res_err;
              r->flags |= DEVICE_RES_FLAGS_ENUM_RESERVED0;
              /* pass fdt phandle instead of pointer, will be changed in resolve_dev_links */
              r->u.uint[0] = endian_be32(*(const uint32_t*)data);
              return;
            }
          else if (!strncmp(name + 1, "aram-array-", 11))
            {
              uintptr_t value[(datalen / 4 + 1) * sizeof(uintptr_t)];
              value[0] = datalen / 4;
              uint_fast8_t i;
              for (i = 0; i < datalen / 4; i++)
                value[i + 1] = endian_be32(((const uint32_t*)data)[i]);
              if (device_res_add_uint_array_param(e->dev, name + 12, value))
                goto res_err;
              return;
            }
       }
      break;

#ifdef CONFIG_DEVICE_IRQ
    case 'i': {
      uint32_t elen = e->interrupt_cells * 4;

      if (!strcmp(name + 1, "nterrupt-parent") && datalen == 4)
        {
          e->icu_phandle = endian_be32(*(const uint32_t*)data);
          return;
        }

      else if (!strcmp(name + 1, "nterrupts") && elen >= 4 && datalen >= elen && e->dev != ctx->dev)
        {
          uint32_t phandle = e->icu_phandle;
          uint32_t j = 0;

          if (phandle == -1)
            break;

          while (datalen >= elen)
            {
              struct dev_resource_s *r;
              error_t err = device_res_alloc(e->dev, &r, DEV_RES_IRQ);
              if (err)
                goto res_err;

              r->u.irq.dev_out_id = j++;
              r->u.irq.icu_in_id = endian_be32(*(const uint32_t*)data8);

              /* logical irq id is passed as second value if interrupt cells size > 4 */
              r->u.irq.irq_id = elen > 4 ? endian_be32(*(const uint32_t*)(data8 + 4)) : 0;

              /* pass fdt phandle instead of pointer, will be changed in resolve_dev_links */
              r->u.uint[0] = phandle;
              r->flags |= DEVICE_RES_FLAGS_ENUM_RESERVED0;

              datalen -= elen;
              data8 += elen;
            }
          return;
        }

      else if (!strcmp(name + 1, "nterrupt-map") && elen >= 4 && datalen >= 8 + elen && e->dev != ctx->dev)
        {
          while (datalen >= 8 + elen)
            {
              struct dev_resource_s *r;
              error_t err = device_res_alloc(e->dev, &r, DEV_RES_IRQ);
              if (err)
                goto res_err;
              r->u.irq.dev_out_id = endian_be32(*(const uint32_t*)data8);
              r->u.irq.icu_in_id = endian_be32(*(const uint32_t*)(data8 + 8));

              /* logical irq id is passed as fourth value if interrupt cells size > 12 */
              r->u.irq.irq_id = elen > 12 ? endian_be32(*(const uint32_t*)(data8 + 12)) : 0;

              /* pass fdt phandle instead of pointer, will be changed in resolve_dev_links */
              r->u.uint[0] = endian_be32(*(const uint32_t*)(data8 + 4));
              r->flags |= DEVICE_RES_FLAGS_ENUM_RESERVED0;

              datalen -= 8 + elen;
              data8 += 8 + elen;
            }
          return;
        }

      else if (!strcmp(name + 1, "nterrupt-controller") && e->dev != ctx->dev)
        {
          return;
        }
    }
#endif

    }

  printk("enum-fdt: device %p `%s', ignored node property `%s : %P'\n", e->dev, e->dev->node.name, name, data, datalen);
  return;

 res_err:
  printk("enum-fdt: device %p `%s', error adding resource entry\n", e->dev, e->dev->node.name);
}

static FDT_ON_MEM_RESERVE_FUNC(enum_fdt_mem_reserve)
{
  //  struct enum_fdt_parse_ctx_s *ctx = priv;
}


DEVENUM_MATCH_DRIVER(enum_fdt_match_driver)
{
  const struct devenum_ident_s *ident = drv->id_table;

  if (!ident)
    return 0;

  for ( ; ident->type != 0; ident++ )
    {
      if (ident->type != DEVENUM_TYPE_FDTNAME)
        continue;

      const struct dev_resource_s *r = device_res_get(dev, DEV_RES_PRODUCT, 0);

      if (!r || !r->u.product.name)
        continue;

      if (!strcmp(ident->fdtname.name, r->u.product.name))
        return 1;
    }

  return 0;
}

static const struct driver_enum_s enum_fdt_enum_drv =
{
  .class_	= DRIVER_CLASS_ENUM,
  .f_match_driver = enum_fdt_match_driver,
};

static DEV_CLEANUP(enum_fdt_cleanup);
static DEV_INIT(enum_fdt_init);

const struct driver_s	enum_fdt_drv =
{
  .desc         = "Flat Device Tree enumerator",
  .f_init	= enum_fdt_init,
  .f_cleanup	= enum_fdt_cleanup,
  .classes	= { &enum_fdt_enum_drv, 0 }
};

static void resolve_dev_links(struct device_s *root, struct device_s *dev)
{
  CONTAINER_FOREACH_NOLOCK(device_list, CLIST, &dev->node.children, {

      if (!(item->flags & DEVICE_FLAG_DEVICE))
        CONTAINER_FOREACH_CONTINUE;

      struct device_s *d = (struct device_s*)item;

      DEVICE_RES_FOREACH(d, r, {

          if (r->flags & DEVICE_RES_FLAGS_ENUM_RESERVED0)
            {
              struct device_s *dep = enum_fdt_get_phandle(root, r->u.uint[0]);

              r->flags ^= DEVICE_RES_FLAGS_ENUM_RESERVED0 | DEVICE_RES_FLAGS_DEPEND;
              /* set path to device or drop resource entry */
              if (dep != NULL)
                {
                  char buf[128], *b = buf;
                  struct device_node_s *p = &d->node;
                  while (p && p != &root->node && b + 3 < buf + sizeof(buf))
                    {
                      p = p->parent;
                      (*b++ = '.'), (*b++ = '.'), (*b++ = '/');
                    }

                  if (p == &root->node && device_get_path(p, b, buf + sizeof(buf) - b, &dep->node, 0) >= 0)
                    {
                      const char *path = strdup(buf);
                      if (path != NULL)
                        {
                          r->u.uint[0] = (uintptr_t)path;
                          r->flags |= DEVICE_RES_FLAGS_FREE_PTR0;
                          continue;
                        }
                    }
                }

              printk("enum-fdt: bad node handle in %p `%s'\n", dep, dep->node.name);
              device_res_cleanup(r);
            }
        });

      resolve_dev_links(root, d);
  });
}

static DEV_INIT(enum_fdt_init)
{
  struct enum_fdt_parse_ctx_s ctx = {
    .dev = dev,
    .stack = {
      {
        .dev = dev,
        .icu_phandle = -1,
        .addr_cells = sizeof(uintptr_t) / 4,
        .size_cells = 1,
#ifdef CONFIG_DEVICE_IRQ
        .interrupt_cells = 1,
#endif
        .section = FDT_SECTION_NONE
      }
    },
    .stack_top = -1,
  };

  struct fdt_walker_s walker = {
    .priv = &ctx,
    .on_node_entry = enum_fdt_node_entry,
    .on_node_leave = enum_fdt_node_leave,
    .on_node_prop = enum_fdt_node_prop,
    .on_mem_reserve = enum_fdt_mem_reserve,
  };

  dev->status = DEVICE_DRIVER_INIT_FAILED;
  dev->drv = &enum_fdt_drv;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  fdt_walk_blob((const void*)addr, &walker);
  resolve_dev_links(dev, dev);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(enum_fdt_cleanup)
{
}

