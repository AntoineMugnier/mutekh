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


VFS_CREATE_CONTEXT(devfs_create_context)
{
  struct devfs_context_s	*ctx = NULL;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_create_context: Initializing DevFS context\n");
#endif

  if ((ctx = mem_alloc(sizeof(struct devfs_context_s), mem_region_get_local(mem_scope_sys))) == NULL)
    return -VFS_ENOMEM;

  // Set private field in vfs_context_s
  context->ctx_pv = ctx;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_create_context: Initializing intern Hash Table\n");
#endif

  devfs_hashfunc_init(&(ctx->hash));

  return 0;
}

VFS_DESTROY_CONTEXT(devfs_destroy_context)
{
  struct devfs_context_s	*dev_ctx = context->ctx_pv;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_create_context: Initializing intern Hash Table\n");
#endif

  devfs_hashfunc_destroy(&dev_ctx->hash);

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printk("devfs_create_context: Initializing DevFS context\n");
#endif

  mem_free(context->ctx_pv);

  return 0;
}

VFS_READ_ROOT(devfs_read_root)
{
  printk("devfs_read_root: This function should not be called\n");
  return 0;
}

VFS_WRITE_ROOT(devfs_write_root)
{
  printk("devfs_write_root: This function should not be called\n");
  return 0;
}
