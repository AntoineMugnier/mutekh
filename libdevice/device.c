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

#include <hexo/error.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <stdio.h>
#include <stdlib.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/types.h>
#include <device/class/enum.h>
#include <hexo/enum.h>

#include <mutek/mem_alloc.h>
#include <mutek/startup.h>

const char dev_pin_driving_e[] = ENUM_DESC_DEV_PIN_DRIVING_E;
const char device_status_e[] = ENUM_DESC_DEVICE_STATUS_E;
const char dev_enum_type_e[] = ENUM_DESC_DEV_ENUM_TYPE_E;

#ifdef CONFIG_DEVICE_TREE

DEV_DECLARE_STATIC(device_enum_root, "root",
                   0, enum_root_drv);

struct device_node_s *device_tree_root()
{
  return &device_enum_root.node;
}

GCT_CONTAINER_PROTOTYPES(device_list, extern inline, device_list,
                         init, destroy, pushback, remove, isempty);

void device_tree_init(void)
{
  /* attach statically declared devices in the device tree */

  extern struct device_s dev_devices_table;
  extern struct device_s dev_devices_table_end;
  struct device_s *d = &dev_devices_table;

  for (d = &dev_devices_table; d < &dev_devices_table_end; d++)
    {
      if (d == &device_enum_root)
        continue;
      device_attach(d, &device_enum_root, d->drv);
    }
}
#endif

static void device_init_(struct device_s *dev)
{
  lock_init(&dev->lock);
  dev->status = DEVICE_INIT_NODRV;
  dev->drv = NULL;
  dev->ref_count = dev->start_count = 0;
  dev->node.flags = DEVICE_FLAG_DEVICE;

#ifdef CONFIG_DEVICE_TREE
  dev->node.name = NULL;
  dev->node.parent = NULL;
  device_list_init(&dev->node.children);
#endif
}

void device_init(struct device_s *dev, const struct dev_resource_table_s *tbl)
{
  device_init_(dev);
  dev->res_tbl = (void*)tbl;
}

static const size_t devsize = align_pow2_up(sizeof(struct device_s), sizeof(uint64_t));

struct device_s *device_alloc(size_t resources)
{
  size_t s = devsize;

  if (resources > 0)
    s += sizeof(struct dev_resource_table_s)
       + sizeof(struct dev_resource_s) * resources;

  struct device_s *dev = mem_alloc(s, (mem_scope_sys));

  if (dev != NULL)
    {
      device_init_(dev);
      dev->node.flags |= DEVICE_FLAG_ALLOCATED;

      if (resources > 0)
        {
          struct dev_resource_table_s *tbl = (void*)((uint8_t*)dev + devsize);

          dev->res_tbl = tbl;
#ifdef CONFIG_DEVICE_RESOURCE_ALLOC
          tbl->next = NULL;
#endif
          tbl->flags = 0;
          tbl->count = resources;

          memset(tbl->table, 0, sizeof(struct dev_resource_s) * resources);
        }
      else
        {
          dev->res_tbl = NULL;
        }
    }

  return dev;
}

#ifdef CONFIG_DEVICE_CLEANUP
void device_cleanup(struct device_s *dev)
{
# ifdef CONFIG_DEVICE_TREE
  assert(!dev->node.parent);
# endif

  assert(dev->status == DEVICE_INIT_FAILED ||
         dev->status == DEVICE_INIT_NODRV);

# ifdef CONFIG_DEVICE_RESOURCE_ALLOC
  struct dev_resource_table_s *next, *tbl;
  for (tbl = dev->res_tbl; tbl != NULL; tbl = next)
    {
      next = tbl->next;
      uint_fast8_t i;
      for (i = 0; i < tbl->count; i++)
        device_res_cleanup(&tbl->table[i]);

      if (tbl->flags & DEVICE_RES_TBL_FLAGS_ALLOCATED)
        mem_free((void*)tbl);
    }
# endif

# ifdef CONFIG_DEVICE_TREE
  device_list_destroy(&dev->node.children);
# endif

  lock_destroy(&dev->lock);

  if (dev->node.flags & DEVICE_FLAG_NAME_ALLOCATED)
    mem_free((void*)dev->node.name);
  if (dev->node.flags & DEVICE_FLAG_ALLOCATED)
    mem_free(dev);
}
#endif

#ifdef CONFIG_DEVICE_RESOURCE_ALLOC
static void device_res_shrink(struct dev_resource_table_s **tbl_,
                              size_t header)
{
  struct dev_resource_table_s *tbl = *tbl_;
  if (tbl == NULL)
    return;
  if (tbl->flags & DEVICE_RES_TBL_FLAGS_STATIC_CONST)
    return;

  if (header || (tbl->flags & DEVICE_RES_TBL_FLAGS_ALLOCATED))
    {
      int_fast8_t i;
      for (i = tbl->count; i > 0; i--)
        if (tbl->table[i-1].type != DEV_RES_UNUSED)
          break;

      size_t s = header;
      if (i < tbl->count)
        {
          if (i)
            {
              tbl->count = i;
              s += sizeof(struct dev_resource_table_s)
                + sizeof(struct dev_resource_s) * i;
              tbl_ = &tbl->next;
            }
          else
            {
              *tbl_ = tbl->next;
            }
          if (s)
            mem_resize((uint8_t*)tbl - header, s);
          else
            mem_free(tbl);
        }
      else
        {
          tbl_ = &tbl->next;
        }
    }

  device_res_shrink(tbl_, 0);
}

void device_shrink(struct device_s *dev)
{
  struct dev_resource_table_s *tbl = dev->res_tbl;

  if ((void*)tbl == (void*)((uint8_t*)dev + devsize) && (dev->node.flags & DEVICE_FLAG_ALLOCATED))
    device_res_shrink(&dev->res_tbl, devsize);
  else
    device_res_shrink(&dev->res_tbl, 0);
}
#endif

error_t device_set_name(struct device_s *dev, const char *name)
{
#ifdef CONFIG_DEVICE_TREE
  if (dev->node.parent)
    return -EBUSY;
#endif

  const char *old = dev->node.name;

  dev->node.name = strdup(name);
  if (!dev->node.name)
    return -ENOMEM;

  if (dev->node.flags & DEVICE_FLAG_NAME_ALLOCATED)
    mem_free((void*)old);

  dev->node.flags |= DEVICE_FLAG_NAME_ALLOCATED;

  return 0;
}

#ifdef CONFIG_DEVICE_TREE

void device_attach(struct device_s *dev,
                   struct device_s *parent,
                   const struct driver_s *drv)
{
  static uint_fast16_t id;
  char name [16];

  assert(!dev->node.parent);

  if (!parent)
    parent = &device_enum_root;

  dev->node.parent = (struct device_node_s*)parent;

  if (!dev->node.name)
    {
      sprintf(name, "dev%u", id++);
      dev->node.name = strdup(name);
      dev->node.flags |= DEVICE_FLAG_NAME_ALLOCATED;
    }

  dev->drv = drv;
  if (drv != NULL)
    dev->status = DEVICE_INIT_PENDING;
#ifdef CONFIG_DEVICE_ENUM
  else if (dev->status == DEVICE_INIT_NODRV)
    dev->status = DEVICE_INIT_ENUM_DRV;
#endif

  device_list_pushback(&parent->node.children, (struct device_node_s*)dev);
}

error_t device_detach(struct device_s *dev)
{
  assert(dev->node.parent);

  if (dev->status != DEVICE_INIT_FAILED &&
      dev->status != DEVICE_INIT_NODRV)
    return -EBUSY;

  device_list_remove(&dev->node.parent->children, &dev->node);
  dev->node.parent = 0;

  return 0;
}

void device_alias_remove(struct device_alias_s *alias)
{
  assert(alias->node.parent);

  device_list_remove(&alias->node.parent->children, &alias->node);

  if (alias->node.flags & DEVICE_FLAG_ALLOCATED)
    {
#ifdef CONFIG_DEVICE_TREE
      if (alias->node.name)
        mem_free((void*)alias->node.name);
      if (alias->path)
        mem_free((void*)alias->path);
#endif

      mem_free(alias);
    }
}

error_t device_get_path(struct device_node_s *root, char *buf, size_t buflen,
                        struct device_node_s *node, uint_fast8_t number)
{
  if (!node)
    return -EINVAL;

  if (node == root)
    {
      if (buflen < 1)
        return -ENOMEM;
      buf[0] = 0;
      return 0;
    }
  else if (node == &device_enum_root.node)
    {
      if (buflen < 2)
        return -ENOMEM;
      buf[0] = '/';
      buf[1] = 0;
      return 1;
    }
  else
    {
      error_t l = device_get_path(root, buf, buflen, node->parent, 0);
      if (l < 0)
        return l;
      buf += l;
      buflen -= l;

      if (l && buf[-1] != '/')
        {
          if (!--buflen)
            return -ENOMEM;
          *buf++ = '/';
          l++;
        }

      const char *name = node->name;
      uint_fast8_t i;
      for (i = 0; i + 1 < buflen && name[i]; i++)
        buf[i] = name[i];

      if (name[i])
        return -ENOMEM;

      if (number == 0)
        {
          buf[i] = 0;
          return l + i;
        }

      ssize_t s = snprintf(buf + i, buflen - i, "[%u]", number);
      if (s >= buflen - i)
        return -ENOMEM;
      return l + i + s;
    }
}

struct device_alias_s * device_new_alias_to_path(struct device_node_s *parent, const char *name, const char *path)
{
  if (!parent)
    parent = &device_enum_root.node;

  struct device_alias_s *alias = mem_alloc(sizeof(struct device_alias_s), (mem_scope_sys));

  if (alias != NULL)
    {
      alias->node.flags = DEVICE_FLAG_ALLOCATED | DEVICE_FLAG_ALIAS;

      device_list_init(&alias->node.children);
      alias->node.parent = parent;

      alias->node.name = strdup(name);

      if (alias->node.name != NULL)
        {
          alias->path = strdup(path);

          if (alias->path != NULL)
            {
              device_list_pushback(&parent->children, &alias->node);
              return alias;
            }

          mem_free((void*)alias->path);
        }

      mem_free((void*)alias->node.name);
    }

  mem_free(alias);

  return NULL;
}

struct device_alias_s * device_new_alias_to_node(struct device_node_s *parent, const char *name, struct device_node_s *node)
{
  char buf[128];

  if (device_get_path(parent, buf, sizeof(buf), node, 0) <= 0)
    return NULL;

  return device_new_alias_to_path(parent, name, buf);
}

error_t device_resolve_alias(struct device_node_s **node, uint_fast8_t depth, const char **brackets)
{
  struct device_node_s *n = *node;
  error_t e = 0;

  while (!e)
    {
      if (!n)
        return -ENOENT;

      if (n->flags & DEVICE_FLAG_ALIAS)
        {
          if (depth < 1)
            return -ELOOP;

          struct device_alias_s *a = (void*)n;
          n = n->parent;
          e = device_node_from_path(&n, a->path, depth - 1, brackets, NULL);
        }
      else
        {      
          *node = n;
          return 0;
        }
    }

  return e;
}

#endif

error_t device_node_from_path(struct device_node_s **node, const char *path,
                              uint_fast8_t depth, const char **brackets,
                              device_filter_t *filter)
{
  __unused__ struct device_node_s *root = *node;
  struct device_node_s *n = NULL;

  while (*path)
    {
      __unused__ struct device_node_s *r = root;

      while (*path && *path <= ' ')
        path++;

#ifdef CONFIG_DEVICE_TREE
      if (*path == '/')
        r = &device_enum_root.node;

      if (brackets)
        *brackets = NULL;

    next:
      if (!r)
        r = &device_enum_root.node;
#endif

      while (*path == '/')
        path++;

      if (path[0] == '.')
        {
          switch (path[1])
            {
            case '/':
              path += 2;
#ifdef CONFIG_DEVICE_TREE
              goto next;
#else
              goto skip;
#endif
            case 0:
            case ' ':
              n = r;
              path += 1;
              if (!n || (filter && !filter(n)))
                goto skip;
              goto end;
            case '.':
#ifdef CONFIG_DEVICE_TREE
              switch (path[2])
                {
                case '/':
                  r = r->parent;
                  path += 3;
                  goto next;
                case 0:
                case ' ':
                  n = r->parent;
                  path += 2;
                  if (!n || (filter && !filter(n)))
                    goto skip;
                  goto end;
                }
#else
              goto skip;
#endif
              break;
            }
        }

      DEVICE_NODE_FOREACH(r, node, {
        uint_fast8_t i;

        if (!node->name)
          continue;

        if (brackets)
          *brackets = NULL;

        for (i = 0; ; i++)
          {
            char c = path[i];
            if (node->name[i] == 0 || c == '*')
              {
                if (c == '*')
                  c = path[++i];
                if (c <= ' ' || (brackets && c == '['))
                  {
                    n = node;
#ifdef CONFIG_DEVICE_TREE
                    error_t e = device_resolve_alias(&n, depth, c == '[' ? NULL : brackets);
                    switch (e)
                      {
                      default:
                        return e;
                      case -ENOENT:
                        n = NULL;
                      case 0:
                        break;
                      }
#endif

                    if (!n || (filter && !filter(n)))
                      break;

                    path += i;
                    goto end;
                  }
                else if (c == '/')
                  {
#ifdef CONFIG_DEVICE_TREE
                    r = node;
                    if (device_resolve_alias(&r, depth, NULL))
                      goto skip;
                    path += i + 1;
                    goto next;
#else
                    goto skip;
#endif
                  }
                break;
              }
            else if (node->name[i] != c && c != '?')
              break;
          }
      });

    skip:
      while (*path > ' ')
        path++;
      continue;

    end:

      if (brackets && *path == '[')
        {
          *brackets = ++path;
          while (*path > ' ')
            path++;
          if (path[-1] != ']')
            return -EINVAL;
        }

      *node = n;
      return 0;
    }

  return -ENOENT;
}

error_t device_get_by_path(struct device_s **dev, uint_fast8_t *number,
                           struct device_node_s *root, const char *path,
                           device_filter_t *filter)
{
  const char *num;
#ifdef CONFIG_DEVICE_TREE
  if (!root)
    root = &device_enum_root.node;
#endif
  error_t e = device_node_from_path(&root, path, 5, &num, filter);
  if (e)
    return e;
  uint_fast8_t n = 0;
  *dev = device_from_node(root);
  if (*dev == NULL)
    return -ENOENT;
  if (num)
    {
      char *end;
      n = strtoul(num, &end, 0);
      if (*end != ']')
        return -EINVAL;
    }
  if (number)
    *number = n;
  return 0;
}

static inline bool_t _device_tree_walk(struct device_node_s *node, device_tree_walker_t *walker, void *priv)
{
  bool_t res = 0;
  struct device_s *d;

  DEVICE_NODE_FOREACH(node, child, {
    if (!(d = device_from_node(child)))
      continue;

    if (walker(d, priv))
      return 1;

#ifdef CONFIG_DEVICE_TREE
    if (_device_tree_walk(child, walker, priv))
      {
        res = 1;
        break;
      }
#endif
  });

  return res;
}

bool_t device_tree_walk(struct device_node_s *root, device_tree_walker_t *walker, void *priv)
{
#ifdef CONFIG_DEVICE_TREE
  if (!root)
    root = &device_enum_root.node;
#endif
  return _device_tree_walk(root, walker, priv);
}

static DEVICE_TREE_WALKER(count_cpus_r)
{
  uint_fast8_t *count = priv;

  if ((dev->node.flags & DEVICE_FLAG_CPU) &&
      dev->status == DEVICE_INIT_DONE)
    (*count)++;

  return 0;
}

uint_fast8_t device_get_cpu_count(void)
{
  uint_fast8_t count = 0;
  device_tree_walk(NULL, count_cpus_r, &count);
  return count;
}

