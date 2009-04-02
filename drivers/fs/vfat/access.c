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
#include <hexo/alloc.h>
#include <mutek/rwlock.h>
#include <vfs/buffer_cache.h>
#include "vfat.h"
#include "vfat-private.h"


#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
uint_fast32_t blk_rd_count;
uint_fast32_t rd_count;
uint_fast32_t wr_count;
#endif


struct bc_request_s* vfat_read_sectors(struct vfat_context_s *ctx, 
				       struct bc_request_s *request,
				       vfat_sector_t first_sector,
				       uint_fast8_t count)
{
  uint8_t *data[1];
  uint_fast32_t i;
 
  request->key1 = (key_t)ctx->dev;
  request->key2 = first_sector;
  request->count = count;
  
  if(bc_get_buffer(&bc,&freelist,request) == NULL)
    goto VFAT_READ_CLUSTER_ERROR;

  for(i=0; i < count; i++)
  {
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    rd_count ++;
#endif
    if(!(IS_BUFFER(request->buffers[i]->state,BC_VALID)))
    {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("-->> reading blk %d..", first_sector + i);
#endif

      data[0] = request->buffers[i]->content;
      
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    blk_rd_count ++;
#endif

      if(dev_block_wait_read(ctx->dev, data, first_sector + i, 1))
	goto VFAT_READ_CLUSTER_ERROR;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("..OK\n");
#endif
    }
  }
   
  return request;
  
 VFAT_READ_CLUSTER_ERROR:
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("I/O Error accured when reading/writing sectors starting at #%d\n",first_sector);
#endif
  bc_release_buffer(&bc,&freelist,request,1);
  return NULL;
}


error_t vfat_write_sector(struct vfat_context_s *ctx,
			  vfat_sector_t sector,
			  uint8_t *content)
{
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];

  request.key1 = (key_t)ctx->dev;
  request.key2 = sector;
  request.count = 1;
  request.buffers = buffers;

  if(bc_get_buffer(&bc,&freelist,&request) == NULL)
    goto VFAT_WRITE_SECTOR_ERROR;

  memcpy(buffers[0]->content, content, ctx->bytes_per_sector);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("'''' write sector %d, ''''\n",sector);
#endif
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
  SET_BUFFER(buffers[0]->state, BC_DELAYED_WRITE);
  bc_release_buffer(&bc,&freelist,&request,0);
  
  return 0;

 VFAT_WRITE_SECTOR_ERROR:
  bc_release_buffer(&bc,&freelist,&request,1);
  return -1;
}


error_t vfat_clear_cluster(struct vfat_context_s *ctx,
			   vfat_sector_t first_sector)
{
  struct bc_request_s request;
  struct bc_buffer_s *buffers[ctx->sectors_per_cluster];
  uint_fast16_t i;
  
  request.key1 = (key_t)ctx->dev;
  request.key2 = first_sector;
  request.count = ctx->sectors_per_cluster;
  request.buffers = buffers;

  if(bc_get_buffer(&bc,&freelist,&request) == NULL)
    goto VFAT_WRITE_SECTOR_ERROR;

  for(i=0; i< request.count; i++)
  {
    memset(buffers[i]->content, 0, ctx->bytes_per_sector);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("'''' write sector %d, ''''\n",first_sector + i);
#endif
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
    SET_BUFFER(buffers[i]->state, BC_DELAYED_WRITE);
  }
  
  bc_release_buffer(&bc,&freelist,&request,0);

  return 0;

 VFAT_WRITE_SECTOR_ERROR:
  bc_release_buffer(&bc,&freelist,&request,1);
  return -1;
}


error_t vfat_query_fat(struct vfat_context_s *ctx,
		       vfat_cluster_t cluster_index, 
		       vfat_cluster_t *next_cluster_index)
{
  vfat_sector_t *sector;
  vfat_cluster_t next_cluster;
  vfat_sector_t lba;
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];
  size_t sector_size;

  sector_size = ctx->bytes_per_sector;
  request.buffers = buffers;
  next_cluster = 0;

  lba =  ctx->fat_begin_lba + ((cluster_index *4) / sector_size);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("Query FAT at %d lba to get next cluster of %d\n", lba, cluster_index);
#endif
 
  if (vfat_read_sectors(ctx, &request, lba, 1) == NULL)
    return -1;

  sector = buffers[0]->content;
  next_cluster = endian_le32_na_load(sector + (cluster_index % (sector_size /4))) & 0x0FFFFFFF;
  bc_release_buffer(&bc,&freelist,&request,0);

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("next cluster for %d is %d\n", cluster_index, next_cluster);
#endif 

  *next_cluster_index = next_cluster;
  return 0;
}


error_t vfat_alloc_fat_entry(struct vfat_context_s *ctx,
			     vfat_cluster_t *new_cluster)
{
  struct bc_request_s request;
  struct bc_buffer_s *buffers[ctx->sectors_per_cluster];
  vfat_sector_t last_allocated_sector;
  uint_fast32_t last_allocated_index;
  uint_fast32_t index;
  vfat_sector_t sector;
  size_t sector_size;
  uint32_t *buffer;
  error_t err;
  bool_t found;

  request.buffers = buffers;
  sector_size = ctx->bytes_per_sector;
  *new_cluster = 0;
  err = 0;
  found = 0;

  rwlock_wrlock(&ctx->lock);
  last_allocated_sector = ctx->last_allocated_sector;
  last_allocated_index = ctx->last_allocated_index;
    
  for(sector = last_allocated_sector; (sector < ctx->fat_blk_count) && (!found); sector ++)
  {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("alloc_fat_entry: sector %d\n",sector);
#endif
    if(vfat_read_sectors(ctx, &request, sector, 1) == NULL)
    {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("alloc_fat: error reading sector %d\n", sector);
#endif
      err = -VFS_IO_ERR;
      goto VFAT_ALLOC_CLUSTER_ERROR;
    }
    buffer = buffers[0]->content;
    for(index = last_allocated_index; index < (sector_size/4); index ++)
    {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
      printk("alloc_fat_entry: index %d\n",index);
#endif
      if((endian_le32_na_load(&buffer[index]) & 0x0FFFFFFF) == 0x00000000)
      {
	found = 1;
	endian_le32_na_store(&buffer[index], 0x0FFFFFF8);
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
	SET_BUFFER(buffers[0]->state,BC_DELAYED_WRITE);
	if(index != ((sector_size/4) - 1))
	  ctx->last_allocated_sector = sector;
	else
	{ 
	  last_allocated_sector = (sector + 1) % ctx->fat_blk_count;
	  ctx->last_allocated_sector = (!last_allocated_sector) ? 
	    ctx->rootdir_first_cluster + 1 : last_allocated_sector;	  
	}
	ctx->last_allocated_index = (index + 1) % (sector_size/4);
	index += (sector - ctx->fat_begin_lba) * (sector_size/4);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
	printk("alloc_fat_entry: found: cluster %d\n",index);
#endif
	*new_cluster = index;
	break;
      }
    }

    last_allocated_index = 0;
    bc_release_buffer(&bc,&freelist,&request,0);
  }

 VFAT_ALLOC_CLUSTER_ERROR: 
  rwlock_unlock(&ctx->lock);
  
  if(found) 
  {
    sector = VFAT_CONVERT_CLUSTER(ctx,*new_cluster);
    err = vfat_clear_cluster(ctx,sector);
  }
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("alloc_entry: new_cluster %d, hasError ? %d\n",*new_cluster,err);
#endif
  return err;
}


error_t vfat_extend_cluster(struct vfat_context_s *ctx, 
			    vfat_cluster_t current_cluster,
			    vfat_cluster_t *next_cluster)
{
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];
  vfat_sector_t lba;
  vfat_cluster_t val;
  size_t sector_size;
  uint32_t *buffer;

  request.buffers = buffers;
  sector_size = ctx->bytes_per_sector;

  if(vfat_alloc_fat_entry(ctx, next_cluster))
    return -1;

  if(*next_cluster == 0)
    return 0;

  lba = ctx->fat_begin_lba + ((current_cluster *4) / sector_size);
  if(vfat_read_sectors(ctx, &request, lba, 1) == NULL)
    return -1;
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG  
  printk("extendig cluster %d\n",current_cluster);
#endif
  buffer = buffers[0]->content;
  val = endian_le32_na_load(&buffer[current_cluster % (sector_size/4)]) & 0x0FFFFFFF;

  if( val >= 0x0FFFFFF8)
    endian_le32_na_store(&buffer[current_cluster % (sector_size/4)], *next_cluster);
  else
  {
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("extend_cluster: %d already extended !\n",current_cluster);
    printk("extend cluster: freeing %d\n",*next_cluster);
#endif
    bc_release_buffer(&bc,&freelist,&request,0);
    lba = ctx->fat_begin_lba + ((*next_cluster *4) / sector_size);
    if(vfat_read_sectors(ctx, &request, lba, 1) == NULL)
      return -2;
    buffer[*next_cluster % (sector_size/4)] = 0x00;
  }
  
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif

  SET_BUFFER(buffers[0]->state, BC_DELAYED_WRITE);
  bc_release_buffer(&bc,&freelist,&request,0);

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("cluster %d's FAT entry is set to %d, FAT's sector %d is set as delayed write\n",
	 current_cluster,*next_cluster,lba);
  printk("allocated cluster %d for asked cluster %d\n",*next_cluster,current_cluster);
#endif
  
  return 0;
}


error_t vfat_free_fat_entry(struct vfat_context_s *ctx, vfat_cluster_t start_cluster)
{
  vfat_sector_t *sector;
  vfat_cluster_t current_index;
  vfat_cluster_t next_index;
  vfat_sector_t lba;
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];
  size_t sector_size;

  sector_size = ctx->bytes_per_sector;
  request.buffers = buffers;
  current_index = start_cluster;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("vfat_free_fat_entry: freeling fat entries starting by %d\n",start_cluster);
#endif


  while((current_index > 0) && (current_index < 0x0FFFFFF7))
  {
    lba =  ctx->fat_begin_lba + ((current_index *4) / sector_size);
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("query FAT at %d lba to get next cluster index of %d\n", 
	   lba, current_index);
#endif

    if (vfat_read_sectors(ctx, &request, lba, 1) == NULL)
      return VFS_IO_ERR;
    
    sector = buffers[0]->content;
    next_index = endian_le32_na_load(&sector[current_index % (sector_size /4)]) & 0x0FFFFFFF;
    sector[current_index % (sector_size /4)] = 0x00;
    SET_BUFFER(buffers[0]->state, BC_DELAYED_WRITE);
#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
    wr_count ++;
#endif
    bc_release_buffer(&bc,&freelist,&request,0);

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
    printk("content of %d is set to 0, next cluster in the list is %d\n", 
	   current_index, next_index);
#endif
    current_index = next_index;
  }
  return 0;
}

error_t vfat_locate_entry(struct vfat_entry_request_s *rq)
{
  struct vfat_context_s *ctx;
  struct vfat_DirEntry_s *dir;
  struct bc_request_s request;
  vfat_cluster_t current_cluster;
  vfat_cluster_t next_cluster;
  vfat_sector_t current_sector;
  vfat_sector_t sector;
  vfat_sector_t entry;
  uint_fast16_t entries_nr;
  
  ctx = rq->ctx;
  struct bc_buffer_s *buffers[ctx->sectors_per_cluster];
  char name[VFS_MAX_NAME_LENGTH + 1]; /* +1 for string null termination */
  bool_t found;
  
  dir = NULL;
  request.buffers = buffers;
  current_cluster = rq->parent_cluster;
  next_cluster = 0;
  current_sector = 0;

  entries_nr = ctx->bytes_per_sector / sizeof(struct vfat_DirEntry_s);
  found = 0;

#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("vfat_locate_entry started: node %s\n",rq->entry_name);
#endif

  while(!found)
  {
    current_sector = VFAT_CONVERT_CLUSTER(ctx,current_cluster);
    
    if(vfat_read_sectors(ctx,&request,current_sector,ctx->sectors_per_cluster) == NULL)
      return VFS_IO_ERR;
    
    for(sector=0; (sector < ctx->sectors_per_cluster) && (!found); sector++)
    {
      dir = buffers[sector]->content;
      for(entry=0; entry < entries_nr; entry ++)
      {
	if(dir[entry].DIR_Name[0] == 0x00)
	{
	  bc_release_buffer(&bc,&freelist,&request,0);
	  return VFS_NOT_FOUND;
	}

	// 0xE5 = long entry
	if(dir[entry].DIR_Name[0] == 0xE5)
	  continue;
	
	if(dir[entry].DIR_Name[0] == '.')
	  continue;	
	
	if(rq->entry_name == NULL)
	  return VFS_ENOTEMPTY;
	
	if(dir[entry].DIR_Attr == 0x0F)
	  continue;

	vfat_getshortname((char *)dir[entry].DIR_Name, name);
       
	if(!strcmp(name,rq->entry_name))
	{
	  found = 1;
	  if(rq->entry != NULL)
	    memcpy(rq->entry,&dir[entry],sizeof(*dir));
	  *rq->entry_sector = sector + current_sector;
	  *rq->entry_index = entry;
	  break;
	}
      }
    }
    bc_release_buffer(&bc,&freelist,&request,0);
    
    if(vfat_query_fat(ctx, current_cluster, &next_cluster))
      return VFS_IO_ERR;
    
    if((next_cluster & 0x0FFFFFFF) == 0x0FFFFFF7)
       return VFS_EBADBLK;
    
    if((next_cluster & 0X0FFFFFFF) >= 0x0FFFFFF8) 
       break;
    
    current_cluster = next_cluster;
  }
#ifdef CONFIG_DRIVER_FS_VFAT_DEBUG
  printk("found %d\n",found);
#endif
  return (found) ? VFS_FOUND : VFS_NOT_FOUND;
}
