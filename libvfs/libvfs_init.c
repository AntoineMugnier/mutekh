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
    Copyright Ghassan Almaless <ghassan.almaless@gmail.com>
*/

#include <mutek/mem_alloc.h>
#include <vfs/vfs.h>
#include <vfs/vfs-private.h>
#include <vfs/buffer_cache.h>
#include <drivers/fs/vfat/vfat.h>

#ifdef CONFIG_DRIVER_FS_PIPE
#include <drivers/fs/pipe/pipe.h>
#endif

#ifdef CONFIG_DRIVER_FS_DEV
#include <drivers/fs/devfs/devfs.h>
#endif

#define BC_BUFFER_SIZE       512
#define BC_ENTRIES_NR         20
#define BC_BUFFERS_PER_ENTRY  3

static struct vfs_node_s __vfs_root;
static struct vfs_node_s *vfs_root = &__vfs_root;

/**
 **
 ** params struct device_s	*device
 ** params ??????????
 ** params ??????????
 ** params struct vfs_node_s	*root
 **
 ** return error_t
 **
 **/

error_t vfs_init (struct device_s * device, uint_fast8_t fs_type,
		  uint_fast8_t node_nr, uint_fast16_t file_nr,
		  struct vfs_node_s ** root)
{
  struct vfs_context_op_s * ctx_op[] = {0,(struct vfs_context_op_s *)&vfat_ctx_op};
  struct vfs_context_s *ctx;
  error_t err;

  assert(device != NULL);
  
  if(fs_type != VFS_VFAT_TYPE)
    {
      printk("vfs_init: invaild fs_type value, this VFS version support only VFAT file system as root file system\n");
      return -VFS_EINVAL;
    }

  memset(vfs_root,0,sizeof(*vfs_root));

  if((err=bc_init(&bc,&freelist,BC_ENTRIES_NR,BC_BUFFERS_PER_ENTRY,BC_BUFFER_SIZE,bc_default_hash)))
    {
      printk("error while initialzing bufferCache :%d\n",err);
      return err;
    }

  if((ctx = mem_alloc(sizeof(*ctx), (mem_scope_sys))) == NULL)
    return -VFS_ENOMEM;

  err = 0;

  ctx->ctx_type = fs_type;
  ctx->ctx_dev = device;

  if((err=ctx_op[fs_type]->create(ctx)))
    return err;

  if((err=vfs_node_init(ctx,vfs_root)))
    return err;

  if((err=ctx_op[fs_type]->read_root(ctx,vfs_root)))
    return err;

  if((err=vfs_node_freelist_init(ctx,node_nr)))
    return err;

  if((err=vfs_file_freelist_init(file_nr)))
    return err;

#ifdef CONFIG_DRIVER_FS_PIPE
  if((vfs_pipe_ctx = mem_alloc(sizeof(*vfs_pipe_ctx), (mem_scope_sys))) == NULL)
    return -VFS_ENOMEM;
  
  memset(vfs_pipe_ctx,0,sizeof(*vfs_pipe_ctx));
  pipe_ctx_op.create(vfs_pipe_ctx);
#endif

  vfs_node_up(vfs_root);

  if (root)
    *root = vfs_root;

  return err;
}

struct vfs_node_s	*vfs_get_root()
{
  return vfs_root;
}

