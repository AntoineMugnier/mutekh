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

#include <hexo/alloc.h>
#include <vfs/vfs.h>
#include <vfs/vfs-private.h>
#include <drivers/fs/vfat/vfat.h>

#ifdef CONFIG_DRIVER_FS_PIPE
#include <drivers/fs/pipe/pipe.h>
#endif

#define BC_BUFFER_SIZE       512
#define BC_ENTRIES_NR         20
#define BC_BUFFERS_PER_ENTRY  3

static struct vfs_node_s __vfs_root;
struct vfs_node_s *vfs_root = &__vfs_root;

VFS_INIT(vfs_init)
{
  struct vfs_context_op_s * ctx_op[] = {0,(struct vfs_context_op_s *)&vfat_ctx_op};
  struct vfs_context_s *ctx;
  error_t err;
  
  assert(root != NULL);
  assert(device != NULL);
  
  if(fs_type != VFS_VFAT_TYPE)
  {
    printf("vfs_init: invaild fs_type value, this VFS version support only VFAT file system as root file system\n");
    return -VFS_EINVAL;
  }

  memset(vfs_root,0,sizeof(*vfs_root));

  if((err=bc_init(&bc,&freelist,BC_ENTRIES_NR,BC_BUFFERS_PER_ENTRY,BC_BUFFER_SIZE,bc_default_hash)))
  {
    printf("error while initialzing bufferCache :%d\n",err);
    return err;
  }

  if((ctx = mem_alloc(sizeof(*ctx), MEM_SCOPE_SYS)) == NULL)
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
  if((vfs_pipe_ctx = mem_alloc(sizeof(*vfs_pipe_ctx), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;
  
  memset(vfs_pipe_ctx,0,sizeof(*vfs_pipe_ctx));
  pipe_ctx_op.create(vfs_pipe_ctx);
#endif

  vfs_node_up(vfs_root);
  *root = vfs_root;
  return err;
}
