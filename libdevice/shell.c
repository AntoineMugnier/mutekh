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

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

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

/* enum option completion handler */
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

#ifdef CONFIG_DEVICE_TREE
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_alias)
{
  device_new_alias_to_path(NULL, argv[0], argv[1]);
  return 0;
}
#endif

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_tree)
{
  device_dump_tree(NULL);
  return 0;
}

extern TERMUI_CON_GROUP_DECL(dev_shell_clock_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_timer_group);
extern TERMUI_CON_GROUP_DECL(dev_shell_mem_group);

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
  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_group) =
{
  TERMUI_CON_GROUP_ENTRY(dev_shell_subgroup, "dev")
};

MUTEK_SHELL_GROUP_REGISTER(dev_shell_group);

