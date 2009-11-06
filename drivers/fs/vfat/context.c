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

#include <device/block.h>
#include <device/driver.h>
#include <mutek/mem_alloc.h>
#include <mutek/rwlock.h>
#include <vfs/buffer_cache.h>
#include "vfat.h"
#include "vfat-private.h"



static inline error_t vfat_context_init(struct vfat_context_s *ctx)
{
  size_t blk_sz = dev_block_getparams(ctx->dev)->blk_size;
  struct vfat_bpb_s *bpb;
  uint8_t *data[1];
  uint8_t d[blk_sz];

  data[0] = d;

  if (dev_block_spin_read(ctx->dev, data, 0, 1))
    {
      printk("VFAT drv error: IO error, couldn't read bpb for device.\n");
      return -1;
    }

  bpb = (struct vfat_bpb_s *)d;

  /* CHECKME/FIXME
     We will take for granted that the physical block size of the
     device must be equivalent to the one provided by the BPB.
  */

  if (blk_sz != (ctx->bytes_per_sector = endian_le16_na_load(&bpb->BPB_BytsPerSec)))
    {
      printk("block size is %d\n",blk_sz);
      printk("++%P++", bpb,512);
      printk("VFAT drv error: bpb/device block size mismatch.\n");

      printk("++++");
      while(1);
      return -1;
    }

  ctx->fat_begin_lba = endian_le16_na_load(&bpb->BPB_RsvdSecCnt);
  ctx->fat_blk_count = endian_le32_na_load(&bpb->BPB_FATSz32);
  ctx->cluster_begin_lba = ctx->fat_begin_lba +
    (bpb->BPB_NumFATs * endian_le32_na_load(&bpb->BPB_FATSz32));
  ctx->sectors_per_cluster = bpb->BPB_SecPerClus;
  ctx->rootdir_first_cluster = endian_le32_na_load(&bpb->BPB_RootClus);
  ctx->bytes_per_cluster = ctx->bytes_per_sector * ctx->sectors_per_cluster;

  ctx->last_allocated_sector = ctx->fat_begin_lba;
  ctx->last_allocated_index = 2;
  
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("\tbegin_lba %d\n \
          blk_count %d\n \
          cluster_begin_lba %d\n \
          sectors_per_cluster %d\n \
          rootdir_first_cluster  %d\n \
          bytes_per_cluster   %d\n",ctx->fat_begin_lba, ctx->fat_blk_count,
	 ctx->cluster_begin_lba, ctx->sectors_per_cluster,
	 ctx->rootdir_first_cluster, ctx->bytes_per_cluster);

  printk("context_init: last allocated sector %d, last allocated index %d\n",
	 ctx->last_allocated_sector, ctx->last_allocated_index);
#endif

  return 0;
}

VFS_CREATE_CONTEXT(vfat_create_context)
{
  struct vfat_context_s *vfat_ctx;
  error_t err;

  err = 0;

  if ((vfat_ctx = mem_alloc(sizeof(*vfat_ctx), mem_region_get_local(mem_scope_sys))) == NULL)
    return VFS_ENOMEM;

  vfat_ctx->dev = context->ctx_dev;
  rwlock_init(&vfat_ctx->lock);

  if ((err = vfat_context_init(vfat_ctx)))
  {
    printk("+++ ERROR INITIALIZING VFAT CONTEXT err %d+++\n",err);
    mem_free(vfat_ctx);
    rwlock_destroy(&vfat_ctx->lock);
    return VFS_EUNKNOWN;
  }

  context->ctx_op = (struct vfs_context_op_s *) &vfat_ctx_op;
  context->ctx_node_op = (struct vfs_node_op_s *) &vfat_n_op;
  context->ctx_file_op = (struct vfs_file_op_s *) &vfat_f_op;
  context->ctx_pv = (void *) vfat_ctx;
  return 0;
}

VFS_DESTROY_CONTEXT(vfat_destroy_context)
{
  struct vfat_context_s *ctx = (struct vfat_context_s *)context->ctx_pv;

  assert(ctx != NULL);
  rwlock_destroy(&ctx->lock);
  mem_free(ctx);
  context->ctx_pv = NULL;
  return 0;
}


VFS_READ_ROOT(vfat_read_root)
{
  struct vfat_context_s *ctx;
  struct vfat_node_s *n_info; 
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];
  vfat_sector_t sector;
  uint_fast8_t attr;

  ctx = (struct vfat_context_s *)context->ctx_pv;
  request.buffers = buffers;
  sector = VFAT_CONVERT_CLUSTER(ctx,ctx->rootdir_first_cluster);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("get root info : asking for blk %d\n", sector);
#endif
  if(vfat_read_sectors(ctx, &request, sector,1) == NULL)
    return VFS_IO_ERR;

  attr=((struct vfat_DirEntry_s *)buffers[0]->content)->DIR_Attr;
  bc_release_buffer(&bc,&freelist,&request,0);
  
  strcpy(root->n_name,"/");
  root->n_size = 0;
  root->n_links = 1;
  root->n_attr |= VFS_DIR;
  
  n_info = root->n_pv;
  n_info->flags = VFAT_ATTR_DIRECTORY;
  n_info->parent_cluster = 0;
  n_info->node_cluster = ctx->rootdir_first_cluster;
  n_info->entry_sector = 0;
  n_info->entry_index = 0;
 
  return 0;
}


VFS_WRITE_ROOT(vfat_write_root)
{
  return 0;
}
