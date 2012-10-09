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

#include <stdio.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/enum.h>

#include <mutek/mem_alloc.h>

#include <mutek/printk.h>

#ifdef CONFIG_DEVICE_TREE
extern const struct driver_s device_enum_root_drv;
struct device_s device_enum_root;
CONTAINER_FUNC(device_list, CLIST, inline, device_list);
#endif

void device_tree_init()
{
#ifdef CONFIG_DEVICE_TREE
  device_init(&device_enum_root);
  device_enum_root.node.name = "root";
  device_bind_driver(&device_enum_root, &device_enum_root_drv);
  device_init_driver(&device_enum_root);
#endif
}

void device_init(struct device_s *dev)
{
  lock_init(&dev->lock);
  dev->status = DEVICE_NO_DRIVER;
  dev->drv = NULL;
  dev->res_count = DEVICE_STATIC_RESOURCE_COUNT;
  dev->ref_count = 0;
  dev->node.flags = DEVICE_FLAG_DEVICE;

#ifdef CONFIG_DEVICE_TREE
  dev->node.name = NULL;
  dev->node.parent = NULL;
  dev->enum_dev = &device_enum_root;
  device_list_init(&dev->node.children);
#endif

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_DEVICE_IRQ)
  //  cpu_set_init(&dev->cpu_irqs);
#endif

  memset(dev->res, 0, sizeof(dev->res));
}

struct device_s *device_alloc(size_t resources)
{
  struct device_s *dev = mem_alloc(sizeof(struct device_s) + sizeof(struct dev_resource_s)
                                   * ((ssize_t)resources - DEVICE_STATIC_RESOURCE_COUNT), (mem_scope_sys));

  if (dev != NULL)
    {
      lock_init(&dev->lock);
      dev->status = DEVICE_NO_DRIVER;
      dev->drv = NULL;
      dev->res_count = resources;
      dev->ref_count = 0;
      dev->node.flags = DEVICE_FLAG_ALLOCATED | DEVICE_FLAG_DEVICE;

#ifdef CONFIG_DEVICE_TREE
      dev->node.name = NULL;
      dev->node.parent = NULL;
      dev->enum_dev = &device_enum_root;
      device_list_init(&dev->node.children);
#endif

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_DEVICE_IRQ)
      //      cpu_set_init(&dev->cpu_irqs);
#endif

      memset(dev->res, 0, sizeof(struct dev_resource_s) * resources);
    }

  return dev;
}

void device_cleanup(struct device_s *dev)
{
  uint_fast8_t i;

#ifdef CONFIG_DEVICE_TREE
  assert(!dev->node.parent);
#endif
  assert(!dev->ref_count);

  if (dev->status == DEVICE_DRIVER_INIT_DONE)
    dev->drv->f_cleanup(dev);

  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;
      switch (r->type)
        {
#ifdef CONFIG_DEVICE_IRQ
        case DEV_RES_IRQ:
          if (r->irq.icu)
            r->irq.icu--;
          break;
#endif
        case DEV_RES_VENDORID:
          if (r->vendor.name && (dev->node.flags & DEVICE_FLAG_ALLOCATED))
            mem_free((void*)r->vendor.name);
          break;

        case DEV_RES_PRODUCTID:
          if (r->product.name && (dev->node.flags & DEVICE_FLAG_ALLOCATED))
            mem_free((void*)r->product.name);
          break;

        case DEV_RES_STR_PARAM:
        case DEV_RES_UINT_ARRAY_PARAM:
          if (dev->node.flags & DEVICE_FLAG_ALLOCATED)
            mem_free((void*)r->str_param.value);
        case DEV_RES_UINT_PARAM:
          if (dev->node.flags & DEVICE_FLAG_ALLOCATED)
            mem_free((void*)r->str_param.name);
          break;

        default:
          break;
        }
    }

#ifdef CONFIG_DEVICE_TREE
  device_list_destroy(&dev->node.children);
#endif

  lock_destroy(&dev->lock);

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_DEVICE_IRQ)
  //  cpu_set_destroy(&dev->cpu_irqs);
#endif

  if (dev->node.flags & DEVICE_FLAG_ALLOCATED)
    {
#ifdef CONFIG_DEVICE_TREE
      if (dev->node.name)
        mem_free((void*)dev->node.name);
#endif

      mem_free(dev);
    }
}

void device_shrink(struct device_s *dev)
{
  uint_fast8_t i;

  assert(dev->node.flags & DEVICE_FLAG_ALLOCATED);

  for (i = dev->res_count; i > 0; i--)
    if (dev->res[i-1].type != DEV_RES_UNUSED)
      break;

  if (i < dev->res_count)
    {
      dev->res_count = i;

      mem_resize(dev, sizeof(struct device_s) + sizeof(struct dev_resource_s)
                 * ((ssize_t)i - DEVICE_STATIC_RESOURCE_COUNT));
    }
}

#ifdef CONFIG_DEVICE_TREE

void device_attach(struct device_s *dev,
                   struct device_s *parent)
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
    }

  device_list_pushback(&parent->node.children, (struct device_node_s*)dev);
}

void device_detach(struct device_s *dev)
{
  assert(dev->node.parent);

  device_list_remove(&dev->node.parent->children, &dev->node);
  dev->node.parent = 0;
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
                        struct device_node_s *node, uint_fast16_t number)
{
  if (!node)
    return -EINVAL;

  if (node == root)
    {
      if (buflen < 1)
        return -ENOMEM;
      buf[1] = 0;
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

          free((void*)alias->path);
        }

      free((void*)alias->node.name);
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

static struct device_node_s *device_resolve_alias(struct device_node_s *node, uint_fast8_t depth, const char **brackets)
{
  while (node && node->flags & DEVICE_FLAG_ALIAS)
    {
      struct device_alias_s *a = (void*)node;
      node = device_node_from_path(node->parent, a->path, depth - 1, brackets);
    }

  return node;
}

struct device_node_s *device_node_from_path(struct device_node_s *root, const char *path,
                                            uint_fast8_t depth, const char **brackets)
{
  struct device_node_s *n = NULL;

  if (!root)
    root = &device_enum_root.node;

  while (*path)
    {
      while (*path && *path <= ' ')
        path++;
      while (*path == '/')
        {
          path++;
          root = &device_enum_root.node;
        }

      const char **b = brackets;
      if (b)
        *b = "";

      // FIXME locking
      CONTAINER_FOREACH_NOLOCK(device_list, CLIST, &root->children,
      {
        uint_fast8_t i;

        if (!item->name)
          CONTAINER_FOREACH_CONTINUE;

        for (i = 0; ; i++)
          {
            char c = path[i];
            if (item->name[i] == 0)
              {
                if (c <= ' ')
                  n = item;
                else if (c == '/')
                  {
                    while (path[i] == '/')
                      i++;
                    n = device_node_from_path(item, path + i, depth, b);
                  }
                else if (c == '[' && b)
                  {
                    *b = path + ++i;
                    b = NULL;
                    while ((c = path[i]) > ' ')
                      i++;
                    if (path[i-1] == ']')
                      n = item;
                  }
                path += i;
                goto end;
              }
            else if (item->name[i] != c && c != '?')
              break;
          }
      });

    end:
      if (depth > 0)
        n = device_resolve_alias(n, depth, b);

      if (n)
        return n;

      while (*path > ' ')
        path++;
    }

  return NULL;
}

struct device_s * device_get_by_path(struct device_node_s *root, const char *path)
{
  const char *unused;
  return device_from_node(device_node_from_path(root, path, 5, &unused));
}

static bool_t _device_tree_walk(struct device_node_s *dev, device_tree_walker_t *walker, void *priv)
{
  bool_t res = 0;

  CONTAINER_FOREACH(device_list, CLIST, &dev->children,
  {
    struct device_s *d = device_from_node(item);

    if(d)
      {
        if (walker(d, priv))
          return 1;

        if (_device_tree_walk(item, walker, priv))
          {
            res = 1;
            CONTAINER_FOREACH_BREAK;
          }
      }
  });

  return res;
}

bool_t device_tree_walk(struct device_node_s *root, device_tree_walker_t *walker, void *priv)
{
  if (!root)
    root = &device_enum_root.node;
  return _device_tree_walk(root, walker, priv);
}

struct device_get_cpu_ctx_s
{
  struct device_s *cpu;
  uint_fast8_t major;
  uint_fast8_t minor;
};

static DEVICE_TREE_WALKER(device_get_cpu_r)
{
  struct device_get_cpu_ctx_s *ctx = priv;

  if (dev->node.flags & DEVICE_FLAG_CPU)
    {
      uintptr_t maj, min;
      if (device_res_get_uint(dev, DEV_RES_ID, 0, &maj, &min))
        return 0;

      if ((ctx->major == maj || ctx->major == -1) && (ctx->minor == min || ctx->minor == -1))
        {
          ctx->cpu = dev;
          return 1;
        }
    }

  return 0;
}

struct device_s *device_get_cpu(uint_fast8_t major_id, uint_fast8_t minor_id)
{
  struct device_get_cpu_ctx_s ctx = { NULL, major_id, minor_id };
  device_tree_walk(NULL, &device_get_cpu_r, &ctx);
  return ctx.cpu;
}

#endif

