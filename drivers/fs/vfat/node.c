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
#include <vfs/buffer_cache.h>
#include "vfat.h"
#include "vfat-private.h"


inline void vfat_getshortname(char *from, char *to)
{
  char *p = to;
  char *q = from;
  uint_fast8_t d;

  for(d=0; (d < 8 && *q != ' '); d++)
    *p++ = *q++;

  if (*(q = &from[8]) != ' ')
    {
      *p++ = '.';
      d = 0;
      while (d++ < 3 && *q != ' ')
	*p++ = *q++;
    }

  *p = 0;
}

static inline void vfat_convert_name(char *str1, char *str2)
{
  uint_fast8_t i;
  char *extention_ptr;

  extention_ptr = str2 + 8;
  memcpy(str2,"           ",11);

  for(i=0; ((*str1 != '.') && (i<11) && (*str1!= '\0')) ; i++)
    *str2++ = *str1++;

  if((i<=8) && (*str1 != '\0'))
    {
      str1++;
      memcpy(extention_ptr,str1,3);
    }
}

VFS_INIT_NODE(vfat_init_node)
{
  struct vfat_node_s *node_info;

  if(node->n_type != VFS_VFAT_TYPE)
    return EINVAL;

  if((node_info = mem_alloc(sizeof(*node_info), MEM_SCOPE_SYS)) == NULL)
    return VFS_ENOMEM;

  memset(node_info, 0, sizeof(*node_info));
  node->n_pv = (void *) node_info;
  return 0;
}


VFS_RELEASE_NODE(vfat_release_node)
{
  if(node->n_pv == NULL)
    return 0;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printf("+++++ vfat_release_node: freeing vfat_node_info\n");
#endif
  mem_free(node->n_pv);
  node->n_pv = NULL;

  return 0;
}


VFS_CREATE_NODE(vfat_create_node)
{
  struct vfat_node_s *node_info;
  struct vfat_node_s *parent_info;
  struct vfat_context_s *ctx;
  struct bc_request_s request;
  struct vfat_DirEntry_s *dir;
  size_t sector_size;
  uint_fast16_t entries_nr;
  vfat_sector_t sector;
  uint_fast16_t entry;
  vfat_cluster_t new_cluster;
  vfat_cluster_t current_cluster;
  vfat_cluster_t next_cluster;
  vfat_sector_t current_sector;

  ctx = (struct vfat_context_s*) parent->n_ctx->ctx_pv;
  struct bc_buffer_s *buffers[ctx->sectors_per_cluster];

  sector_size = ctx->bytes_per_sector;
  entries_nr = sector_size / sizeof(struct vfat_DirEntry_s);
  dir = NULL;
  node_info = NULL;
  parent_info = NULL;
  new_cluster = 0;
  next_cluster = 0;
  request.buffers = buffers;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printf("vfat_create_node started, node to be create %s, it's parent %s, sector size %d\n",
	 node->n_name,parent->n_name,sector_size);
#endif

  if(node->n_type != VFS_VFAT_TYPE)
    return VFS_EINVAL;

  if(!(node->n_attr & VFS_FIFO))
  {
    if(vfat_alloc_fat_entry(ctx, &new_cluster))
      return VFS_IO_ERR;

    if(new_cluster == 0)
      return -VFS_ENOSPC;
  }
  node_info = node->n_pv;
  parent_info =(struct vfat_node_s*) parent->n_pv;
  current_cluster = parent_info->node_cluster;

  while(1)
  {
    current_sector = VFAT_CONVERT_CLUSTER(ctx,current_cluster);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printf("vfat_create_node: current sector %d, cluster %d, parent node %s\n",
	   current_sector,current_cluster,parent->n_name);
#endif

    if(vfat_read_sectors(ctx, &request, current_sector, ctx->sectors_per_cluster) == NULL)
      return VFS_IO_ERR;	/* FIXME: we should free new_cluster */

    for(sector=0; sector < ctx->sectors_per_cluster; sector++)
    {
      dir = buffers[sector]->content;
      for(entry=0; entry < entries_nr; entry ++)
	if((dir[entry].DIR_Name[0] == 0x00) || (dir[entry].DIR_Name[0] == 0xE5)){
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
	  printf("create: found entry %d in sector %d where current cluster is %d and cluster's sector is %d, name[0] %d\n",
		 entry,current_sector,current_cluster,sector, dir[entry].DIR_Name[0]);
#endif
	  goto FREE_ENTRY_FOUND;
	}
    }

    bc_release_buffer(&bc,&freelist,&request,0);

    if(vfat_query_fat(ctx, current_cluster, &next_cluster))
      return VFS_IO_ERR;

    if((next_cluster & 0X0FFFFFFF) >= 0x0FFFFFF8) break;
    current_cluster = next_cluster;
  }

  if(vfat_extend_cluster(ctx, current_cluster, &next_cluster))
    return VFS_EUNKNOWN;       /* FIXME: we should free next_cluster */

  if(next_cluster == 0)
    return VFS_EUNKNOWN;

  current_sector = VFAT_CONVERT_CLUSTER(ctx, next_cluster);

  if(vfat_read_sectors(ctx, &request, current_sector, ctx->sectors_per_cluster) == NULL)
    return VFS_IO_ERR;          /* FIXME: we should free next_cluster */

  for(sector=0; sector < ctx->sectors_per_cluster; sector ++)
  {
    memset(buffers[sector]->content, 0, sector_size);
    SET_BUFFER(buffers[sector]->state, BC_DELAYED_WRITE);

#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
  }

  dir = buffers[0]->content;
  entry = 0;
  sector = 0;

 FREE_ENTRY_FOUND:
  vfat_convert_name(node->n_name,(char *)dir[entry].DIR_Name);  /* FIXME: name may be long */

  dir[entry].DIR_FstClusHI = new_cluster >> 16;
  dir[entry].DIR_FstClusLO = new_cluster & 0xFFFF;
  dir[entry].DIR_FileSize = 0;
  dir[entry].DIR_Attr = 0;
  if(node->n_attr & VFS_DIR)
  {
    dir[entry].DIR_Attr |= VFAT_ATTR_DIRECTORY;
    node_info->flags = VFAT_ATTR_DIRECTORY;
  }
  if((node->n_attr & VFS_SYS) || (node->n_attr & VFS_FIFO))
    dir[entry].DIR_Attr |= VFAT_ATTR_SYSTEM;

  if(node->n_attr & VFS_ARCHIVE)
    dir[entry].DIR_Attr |= VFAT_ATTR_ARCHIVE;

  if((node->n_attr & VFS_RD_ONLY) || (node->n_attr & VFS_FIFO))
    dir[entry].DIR_Attr |= VFAT_ATTR_READ_ONLY;

  SET_BUFFER(buffers[sector]->state, BC_DELAYED_WRITE);
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
  bc_release_buffer(&bc,&freelist,&request,0);

  node_info->parent_cluster = parent_info->node_cluster;
  node_info->node_cluster = new_cluster;
  node_info->entry_sector = sector + current_sector;
  node_info->entry_index = entry;

  node->n_pv = (void *) node_info;
  return 0;
}


VFS_LOOKUP_NODE(vfat_lookup_node)
{
  struct vfat_context_s *ctx;
  struct vfat_node_s *parent_info;
  struct vfat_node_s *node_info;
  struct vfat_entry_request_s rq;
  struct vfat_DirEntry_s dir;
  vfat_sector_t entry_sector;
  vfat_sector_t entry_index;
  error_t err;

  ctx = (struct vfat_context_s*) parent->n_ctx->ctx_pv;
  parent_info = parent->n_pv;
  node_info = node->n_pv;
  err = 0;

  if(!(parent_info->flags & VFAT_ATTR_DIRECTORY))
    return VFS_ENOTDIR;

  rq.ctx = ctx;
  rq.parent_cluster = parent_info->node_cluster;
  rq.entry_name = node->n_name;
  rq.entry = &dir;
  rq.entry_sector = &entry_sector;
  rq.entry_index = &entry_index;

  if((err=vfat_locate_entry(&rq)))
    return err;

  if(((node->n_attr & VFS_DIR) && 1) ^ ((dir.DIR_Attr & VFAT_ATTR_DIRECTORY) && 1))
    return VFS_ENOTDIR;

  if(dir.DIR_Attr & VFAT_ATTR_DIRECTORY)
    node->n_attr |= VFS_DIR;
  else
    node->n_size = endian_le32_na_load(&dir.DIR_FileSize);

  if(dir.DIR_Attr & VFAT_ATTR_SYSTEM)    node->n_attr |= VFS_SYS;
  if(dir.DIR_Attr & VFAT_ATTR_ARCHIVE)   node->n_attr |= VFS_ARCHIVE;
  if(dir.DIR_Attr & VFAT_ATTR_READ_ONLY) node->n_attr |= VFS_RD_ONLY;
  node->n_links = 1;
  node_info->flags = dir.DIR_Attr;
  node_info->parent_cluster = parent_info->node_cluster;
  node_info->node_cluster = endian_le16_na_load(&dir.DIR_FstClusHI) << 16;
  node_info->node_cluster |= (0x0000FFFF & endian_le16_na_load(&dir.DIR_FstClusLO));
  node_info->entry_sector = entry_sector;
  node_info->entry_index = entry_index;
  if((!node_info->node_cluster) && (node->n_attr & VFS_SYS) && (node->n_attr & VFS_RD_ONLY))
    node->n_attr |= VFS_FIFO;

  return VFS_FOUND;
}

VFS_WRITE_NODE(vfat_write_node)
{
  struct bc_request_s request;
  struct vfat_context_s *ctx;
  struct vfat_node_s *node_info;
  struct vfat_DirEntry_s *dir;
  struct bc_buffer_s *buffers[1];
  vfat_sector_t entry_sector;
  uint_fast16_t entry;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printf("write node %s started\n",node->n_name);
#endif

  ctx = node->n_ctx->ctx_pv;
  request.buffers = buffers;
  node_info = node->n_pv;
  entry_sector = node_info->entry_sector;
  entry = node_info->entry_index;

  if(node->n_type != VFS_VFAT_TYPE)
    return VFS_EINVAL;

  if(vfat_read_sectors(ctx, &request, entry_sector, 1) == NULL)
    return VFS_IO_ERR;

  dir = buffers[0]->content;

  if(node->n_attr & VFS_DIR) dir[entry].DIR_Attr |= VFAT_ATTR_DIRECTORY;
  if(node->n_attr & VFS_SYS) dir[entry].DIR_Attr |= VFAT_ATTR_SYSTEM;
  if(node->n_attr & VFS_ARCHIVE) dir[entry].DIR_Attr |= VFAT_ATTR_ARCHIVE;
  if(node->n_attr & VFS_RD_ONLY) dir[entry].DIR_Attr |= VFAT_ATTR_READ_ONLY;

  dir[entry].DIR_FileSize = node->n_size;
  SET_BUFFER(buffers[0]->state, BC_DELAYED_WRITE);
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
  bc_release_buffer(&bc,&freelist,&request,0);

  return 0;
}

VFS_UNLINK_NODE(vfat_unlink_node)
{
  struct vfat_entry_request_s rq;
  struct vfs_node_s fake_node;
  struct bc_request_s request;
  struct vfat_context_s *ctx;
  struct vfat_node_s *node_info;
  struct vfat_node_s *parent_info;
  struct vfat_DirEntry_s *dir;
  struct bc_buffer_s *buffers[1];
  vfat_sector_t entry_sector;
  vfat_sector_t entry_index;
  error_t err;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printf("vfat_unlink node %s started, n_links %d, n_count %d\n",
	 node->n_name,node->n_links,node->n_count);
#endif

  err = 0;
  ctx = node->n_parent->n_ctx->ctx_pv;
  request.buffers = buffers;
  node_info = node->n_pv;
  parent_info = node->n_parent->n_pv;
  memset(&fake_node,0,sizeof(fake_node));

  if((node->n_count) && (node->n_attr & VFS_FIFO) && (node->n_op != &vfat_n_op))
  {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printf("vfat_unlink: node is a fifo, locating it in it's parent directory\n");
#endif

    rq.ctx = ctx;
    rq.parent_cluster = parent_info->node_cluster;
    rq.entry_name = node->n_name;
    rq.entry = NULL;
    rq.entry_sector = &entry_sector;
    rq.entry_index = &entry_index;

    if((err=vfat_locate_entry(&rq)))
      return err;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printf("vfat_unlink: node located at sector %d, entry %d\n",entry_sector,entry_index);
#endif
  }
  else
  {
    entry_sector = node_info->entry_sector;
    entry_index = node_info->entry_index;
  }

  if(node->n_count)
  {
    if(vfat_read_sectors(ctx, &request, entry_sector, 1) == NULL)
      return VFS_IO_ERR;

    dir = buffers[0]->content;

    dir[entry_index].DIR_Name[0] = 0xE5;
    SET_BUFFER(buffers[0]->state, BC_DELAYED_WRITE);

#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printf("sector %d is set to delayed write\n",entry_sector);
#endif
    bc_release_buffer(&bc,&freelist,&request,0);
  }
  else
    if(!(node->n_attr & VFS_FIFO))
      err=vfat_free_fat_entry(ctx,node_info->node_cluster);

  return err;
}
