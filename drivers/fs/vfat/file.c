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
#include <vfs/buffer_cache.h>
#include "vfat.h"
#include "vfat-private.h"

VFS_OPEN_FILE(vfat_open)
{
  struct vfat_context_s *ctx;
  struct vfat_node_s *node_info;
  struct vfat_file_s *file_info;

  ctx = node->n_ctx->ctx_pv;
  node_info = node->n_pv;
  file_info = file->f_pv;
  
  if(file_info == NULL)
    if((file_info = mem_alloc(sizeof(*file_info), mem_region_get_local(mem_scope_sys))) == NULL)
      return VFS_ENOMEM;

  file_info->ctx = ctx;
  file_info->node_cluster = node_info->node_cluster;
  file_info->current_cluster = node_info->node_cluster;
  file_info->current_offset = 0;
  file->f_pv = file_info;
  return 0;
}

VFS_READ_FILE(vfat_read)
{
  struct vfat_context_s *ctx;
  struct vfat_file_s *file_info;
  struct bc_request_s request;
  vfat_cluster_t next_cluster;
  vfat_cluster_t current_cluster;
  vfat_offset_t current_cluster_offset;
  vfat_offset_t current_sector_offset;
  vfat_sector_t current_sector;
  vfat_sector_t first_sector;
  uint8_t *pread;
  uint8_t *pbuff;
  size_t asked_size;
  size_t sector_size;
  uint_fast32_t i,count;
  int_fast32_t bytes_left;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG   
  printk("++++ vfat_read started, asked size %d\n", size);
#endif

  if(file->f_node->n_attr & VFS_FIFO)
    return -EINVAL;
  
  if(size == 0) return 0;

  file_info = file->f_pv;
  ctx = file_info->ctx;
  struct bc_buffer_s *buffers[ctx->sectors_per_cluster];
  request.buffers = buffers;
  sector_size = ctx->bytes_per_sector;
  asked_size =  size;
  pbuff = buffer;
  current_cluster = file_info->current_cluster;
  current_cluster_offset = file_info->current_offset;

  while(size > 0)
  { 
    if(current_cluster_offset >= ctx->bytes_per_cluster)
    {
      if(vfat_query_fat(ctx, current_cluster, &next_cluster))
	return -VFS_IO_ERR;  

      if(current_cluster == 0x0FFFFFF7)
	return -VFS_EBADBLK;

      if (next_cluster >= 0x0FFFFFF8)
	break;

      current_cluster = next_cluster;
      current_cluster_offset = 0;
    }
    first_sector = VFAT_CONVERT_CLUSTER(ctx,current_cluster);
    i = current_cluster_offset / sector_size;
    current_sector = first_sector + i ;
    count = ctx->sectors_per_cluster - i;

    if(vfat_read_sectors(ctx, &request, current_sector, count) == NULL)
      return -VFS_IO_ERR;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG   
    printk("vfat_read: cluster %d, sector %d, current_sector %d, i %d, count %d, cluster_offset %d\n",
	   current_cluster, first_sector, current_sector, i , count,current_cluster_offset);
#endif
    i = 0;
    while((size > 0) && (i < count))
    {
      current_sector_offset = current_cluster_offset % sector_size;
      pread = buffers[i]->content + current_sector_offset;
      bytes_left = sector_size - current_sector_offset;

      if(size >= bytes_left)
      {
	memcpy(pbuff, pread, bytes_left);

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG   
	printk("size >= bytes_left, bytes_left %d, size %d\n",bytes_left,size);
#endif
	pbuff += bytes_left;
	size -= bytes_left;
	current_cluster_offset += bytes_left;
	i++;
      }
      else
      {
	memcpy(pbuff, pread, size);
	
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG   
	printk("size < bytes_left, bytest_left %d, size %d\n",bytes_left,size);
#endif
	current_cluster_offset += size;
	size = 0;
      }
    }

    bc_release_buffer(&bc,&freelist,&request,0);
  }
  
  file_info->current_cluster = current_cluster;
  file_info->current_offset = current_cluster_offset;

  return asked_size - size;
}


VFS_WRITE_FILE(vfat_write)
{
  struct vfat_context_s *ctx;
  struct vfat_file_s *file_info;
  struct vfat_node_s *node_info;
  struct bc_request_s request;
  vfat_cluster_t next_cluster;
  vfat_cluster_t current_cluster;
  vfat_offset_t current_cluster_offset;
  vfat_offset_t sector_offset;
  vfat_sector_t current_sector;
  vfat_sector_t first_sector;
  uint8_t *pwrite;
  uint8_t *pbuff;
  size_t asked_size;
  size_t sector_size;
  int_fast32_t bytes_left;
  error_t err;

  if(file->f_node->n_attr & VFS_FIFO)
    return -EINVAL;
   
   if(size == 0) return 0;

  file_info = file->f_pv;
  node_info = file->f_node->n_pv;
  ctx = file_info->ctx;
  struct bc_buffer_s *buffers[1];
  request.buffers = buffers;
  sector_size = ctx->bytes_per_sector;
  asked_size =  size;
  pbuff = buffer;
  current_cluster = file_info->current_cluster;
  current_cluster_offset = file_info->current_offset;
  err = 0;

  if(!node_info->node_cluster)
  {
    if((err=vfat_alloc_fat_entry(ctx,&current_cluster)))
      return err;
    
    if(!current_cluster)
      return -VFS_ENOSPC;

    node_info->node_cluster = current_cluster;
    file_info->node_cluster = current_cluster;
  }


#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("++ write started for cluster %d, size %d\n",current_cluster,size);
#endif


  while (size > 0)
  {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("main loop, start cluster : %d, current size %d, current_offset %d\n", 
	   current_cluster,size,current_cluster_offset);
#endif
    if(current_cluster_offset >= ctx->bytes_per_cluster)
    {
      if(vfat_query_fat(ctx, current_cluster,&next_cluster)) 
	return -VFS_IO_ERR;  
      
      if(next_cluster == 0x0FFFFFF7)
	return -VFS_EBADBLK;
      
      if(next_cluster >= 0x0FFFFFF8)
	if(vfat_extend_cluster(ctx,current_cluster,&next_cluster))
	  return -VFS_IO_ERR;      
      
      if(next_cluster == 0){
	if(asked_size == size) return -VFS_ENOSPC; else break;
      }
     
      current_cluster = next_cluster;
      current_cluster_offset = 0;
    }

    first_sector = VFAT_CONVERT_CLUSTER(ctx, current_cluster);
    current_sector = first_sector + current_cluster_offset / sector_size;
    sector_offset = current_cluster_offset % sector_size;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("current_sector %d, sector_offset %d, size %d\n",current_sector,sector_offset,size);
#endif
    if((sector_offset == 0) && (size >= sector_size))
    {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("writing complet blk\n");
#endif
      if(vfat_write_sector(ctx,current_sector,pbuff))
	return -VFS_IO_ERR;
	
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("written complet blk\n");
#endif
      size -= sector_size;
      pbuff += sector_size;
      current_cluster_offset += sector_size;
      continue;
    }

    bytes_left = sector_size - sector_offset;
      
    if(vfat_read_sectors(ctx,&request,current_sector,1) == NULL)
      return -VFS_IO_ERR;

    SET_BUFFER(buffers[0]->state, BC_DELAYED_WRITE);
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
    pwrite = buffers[0]->content + sector_offset;

    if(size >= bytes_left)
    {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("size >= bytes_left, size %d, writing %d bytes\n",size,bytes_left);
#endif
      memcpy(pwrite,pbuff, bytes_left);
      pbuff += bytes_left;
      size -= bytes_left;
      current_cluster_offset += bytes_left;
    }
    else
    {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("size < bytes_left, bytes_left %d, writing %d bytes\n",bytes_left,size);
#endif
      memcpy(pwrite, pbuff, size);
      current_cluster_offset += size;
      size = 0;
    }
    
    bc_release_buffer(&bc,&freelist,&request,0); 
  }

  file_info->current_cluster = current_cluster;
  file_info->current_offset = current_cluster_offset;
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("end of write function : written %d\n",asked_size - size);
#endif
  return asked_size - size;
}

VFS_LSEEK_FILE(vfat_lseek)
{
  struct vfat_context_s *ctx;
  struct vfat_file_s *file_info;
  vfat_cluster_t current_cluster;
  vfat_cluster_t next_cluster;
  vfat_cluster_t cluster_rank,i;
  vfat_offset_t cluster_offset;
  
  file_info = file->f_pv;
  ctx = file_info->ctx;

  if(file->f_node->n_attr & VFS_FIFO)
    return -EINVAL;
  
  cluster_rank = (vfat_offset_t)file->f_offset / ctx->bytes_per_cluster;
  cluster_offset = (vfat_offset_t)file->f_offset % ctx->bytes_per_cluster;

  current_cluster = file_info->node_cluster;
  next_cluster = 0;
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("lseek started, clus rank %d\n",cluster_rank);
#endif
  for(i=0; i < cluster_rank; i++)
  {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("lssek: loop %d/%d\n",i,cluster_rank);
#endif
    if(vfat_query_fat(ctx, current_cluster,&next_cluster)) 
      return -VFS_IO_ERR;  
  
    if(next_cluster == 0x0FFFFFF7)
      return -VFS_EBADBLK;
    
    if(next_cluster >= 0x0FFFFFF8)
      if(vfat_extend_cluster(ctx,current_cluster,&next_cluster))
	return -VFS_IO_ERR;
    
    if(next_cluster == 0) 
	return -VFS_ENOSPC;
    
    current_cluster = next_cluster;
  }

  file_info->current_cluster = current_cluster;
  file_info->current_offset = cluster_offset;

  return 0;
}

VFS_RELEASE_FILE(vfat_release)
{
  if(file->f_pv != NULL)
  {
    mem_free(file->f_pv);
    file->f_pv = NULL;
  }
  return 0;
}

VFS_READ_DIR(vfat_readdir)
{
  struct vfat_context_s *ctx;
  struct vfat_file_s *file_info;
  struct bc_request_s request;
  struct vfat_DirEntry_s dir;
  vfat_cluster_t current_cluster;
  vfat_cluster_t node_cluster;
  vfat_offset_t current_offset;
  vfat_offset_t cluster_offset;
  vfat_cluster_t next_cluster;
  uint8_t *current_sector;
  vfat_sector_t sector;
  error_t err;
  bool_t found;

  file_info = file->f_pv;
  ctx = file_info->ctx;
  struct bc_buffer_s *buffers[1];

  err = 0;
  found = 0;
  next_cluster = 0;
  node_cluster = 0;
  current_sector = NULL;
  request.buffers = buffers;
  current_cluster = file_info->current_cluster;
  cluster_offset = file_info->current_offset;
  
  if(cluster_offset >= ctx->bytes_per_cluster)
    return VFS_EODIR;
  
  while(!found)
  {
    sector = VFAT_CONVERT_CLUSTER(ctx,current_cluster);
    
    sector += cluster_offset / ctx->bytes_per_sector;
    current_offset = cluster_offset % ctx->bytes_per_sector;
    
    if(vfat_read_sectors(ctx, &request, sector, 1) == NULL)
      return VFS_IO_ERR;
    
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("vfat_readdir: sector %d, offset %d, cluster %d, offset %d\n",
	   sector,current_offset,current_cluster,cluster_offset);
#endif
    current_sector = buffers[0]->content;
    memcpy(&dir,current_sector + current_offset, sizeof(dir));
    bc_release_buffer(&bc,&freelist,&request,0);
    
    if(dir.DIR_Name[0] == 0x00){
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("vfat_readdir: entries termination found (0x00)\n");
#endif
      goto VFS_READ_DIR_EODIR;
    }
    if(dir.DIR_Name[0] == 0xE5){
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("entry was freeed previously\n");
      vfat_getshortname((char*)dir.DIR_Name,dirent->d_name);
      printk("it was %s\n",dirent->d_name);
#endif
      goto VFS_READ_DIR_NEXT;
    }
    if(dir.DIR_Attr == 0x0F){
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("this entry is a long one\n");
      vfat_getshortname((char*)dir.DIR_Name,dirent->d_name);
      printk("trying to read it's name %s\n",dirent->d_name);
#endif
      goto VFS_READ_DIR_NEXT;
    }
    if(dir.DIR_Name[0] == '.')
      goto VFS_READ_DIR_NEXT;

    found = 1;
    vfat_getshortname((char *)dir.DIR_Name, dirent->d_name);
    dirent->d_size = endian_le32_na_load(&dir.DIR_FileSize);
    dirent->d_type = 0;
    if(dir.DIR_Attr & VFAT_ATTR_DIRECTORY)
      dirent->d_type = VFS_DIR;
    
    if(dir.DIR_Attr & VFAT_ATTR_SYSTEM)     
      dirent->d_type |= VFS_SYS;
    
    if(dir.DIR_Attr & VFAT_ATTR_ARCHIVE)    
      dirent->d_type |= VFS_ARCHIVE;
    
    if(dir.DIR_Attr & VFAT_ATTR_READ_ONLY)  
      dirent->d_type |= VFS_RD_ONLY;
   
    node_cluster = endian_le16_na_load(&dir.DIR_FstClusHI) << 16;
    node_cluster |= (0x0000FFFF & endian_le16_na_load(&dir.DIR_FstClusLO));

    if((!node_cluster) && (dirent->d_type == (VFS_RD_ONLY | VFS_SYS)))
      dirent->d_type |= VFS_FIFO;

  VFS_READ_DIR_NEXT:
    cluster_offset += sizeof(struct vfat_DirEntry_s);
    if(cluster_offset >= ctx->bytes_per_cluster)
    {
      if(vfat_query_fat(ctx, current_cluster, &next_cluster))
        return VFS_IO_ERR;
      
      if((next_cluster & 0x0FFFFFFF) == 0x0FFFFFF7){
	return VFS_EBADBLK;
      }
      
      if((next_cluster & 0X0FFFFFFF) >= 0x0FFFFFF8)
	goto VFS_READ_DIR_EODIR;
      
      current_cluster = next_cluster;
      cluster_offset = 0;
    }
  }

 VFS_READ_DIR_EODIR:
  file_info->current_cluster = current_cluster;
  file_info->current_offset = cluster_offset;

  return (found) ? 0 : VFS_EODIR;
}
