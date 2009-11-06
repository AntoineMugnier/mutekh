/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    UPMC / LIP6 / SOC (c) 2008
    Copyright Sylvain Leroy <sylvain.leroy@unmondelibre.fr>
*/

#include <mutek/mem_alloc.h>
#include "devfs.h"

VFS_INIT_NODE(devfs_init_node)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_init_node: you shoud not see that\n");
#endif

  return 0;
}

VFS_CREATE_NODE(devfs_create_node)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_create_node: trying to create %s\n", node->n_name);
#endif

  return 0;
}

VFS_LOOKUP_NODE(devfs_lookup_node)
{
  struct devfs_context_s	*ctx = NULL;
  struct devfs_node_s		*new_node = NULL;


#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_lookup_node: lookup for %s\n", node->n_name);
#endif

  ctx = devfs_get_ctx();

  if ((new_node = devfs_hashfunc_lookup(&(ctx->hash), node->n_name)) == NULL)
      return DEVFS_ERR;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_lookup_node: %s found\n", node->n_name);
#endif

  return DEVFS_OK;
}

VFS_WRITE_NODE(devfs_write_node)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_write_node: you shoud not see that\n");
#endif

    return 0;
}

VFS_RELEASE_NODE(devfs_release_node)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_release_node: releasing...\n");
#endif

  return 0;
}

VFS_UNLINK_NODE(devfs_unlink_node)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_unlink_node: unlinking...\n");
#endif

  return 0;
}
