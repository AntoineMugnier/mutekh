#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include "vfat.h"
#include "vfat_private.h"

error_t vfat_read_cluster(struct vfat_disk_context_s *ctx,
			  size_t clus_offset,
			  uint8_t *clus_buffer)
{
  struct dev_block_rq_s rq;
  uint8_t *data[ctx->sectors_per_cluster];
  uint32_t i;

  rq.lba = ctx->cluster_begin_lba +
    (clus_offset - 2) * ctx->sectors_per_cluster;
  rq.count = ctx->sectors_per_cluster;
  rq.data = data;

  for (i = 0; i < ctx->sectors_per_cluster; i++)
    data[i] = clus_buffer + i * ctx->bytes_per_sector;

  return (dev_block_wait_read(ctx->fs.device, &rq));
}

error_t vfat_read_blk(struct vfat_disk_context_s *ctx,
		      dev_block_lba_t lba,
		      uint8_t *blk_buff)
{
  struct dev_block_rq_s rq;
  uint8_t *data[1];

  rq.lba = lba;
  rq.count = 1;
  rq.data = data;
  data[0] = (uint8_t *)blk_buff;

  return (dev_block_wait_read(ctx->fs.device, &rq));
}

uint32_t vfat_query_fat(struct vfat_disk_context_s *ctx,
			 uint32_t clus_idx)
{
  /* fat entries are 32 bits wide, hence we devide bps by 4 */
  uint32_t buffer[ctx->bytes_per_sector / 4];

  if (vfat_read_blk(ctx, ctx->fat_begin_lba + (clus_idx / ctx->bytes_per_sector), (uint8_t *)buffer))
    return 0;
  return (*(buffer + (clus_idx % ctx->bytes_per_sector)) & 0x0FFFFFFF);
}

VFS_EXPORT error_t vfat_read(struct fs_disk_context_s *disk_context,
			     struct fs_handle_s *handle,
			     uint8_t *buffer,
			     uint32_t size)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  uint8_t cluster[ctx->bytes_per_cluster];
  struct vfat_handle_s *hndl = handle->pv;
  uint8_t *pbuff = buffer;
  uint8_t *pclus;

  /* get the number of clusters to read in order to fill the given buffer */
  uint32_t clus_to_read = ((hndl->clus_offset + size) / (ctx->bytes_per_cluster + 1)) + 1;

  uint32_t d;
  uint32_t r = size;

  /* for each cluster, fetch data and copy it to buffer */
  while (clus_to_read--)
    {
      if (hndl->clus_idx < 0x0ffffff7)
	{
	  pclus = cluster + hndl->clus_offset;
	  vfat_read_cluster(ctx, hndl->clus_idx, cluster);

	  if (clus_to_read)
	    {
	      d = ctx->bytes_per_cluster - hndl->clus_offset;
	      memcpy(pbuff, pclus, d);
	      hndl->clus_offset = 0;
	      hndl->clus_idx = vfat_query_fat(ctx, hndl->clus_idx);
	      r -= d;
	    }
	  else
	    {
	      memcpy(pbuff, pclus, r);
	      if (hndl->clus_offset + r < ctx->bytes_per_cluster)
		hndl->clus_offset += r;
	      else
		{
		  hndl->clus_offset = 0;
		  hndl->clus_idx = vfat_query_fat(ctx, hndl->clus_idx);
		}
	    }
	  pbuff += d;
	}
      else
	{
          /* test the EndOfBlock tag */
          if (hndl->clus_idx >= 0x0ffffff8)
            return (FS_ERROR_EOF);
          return (FS_ERROR_BADBLK);
	}
    }
  return FS_OK;
}

VFS_EXPORT error_t vfat_write(struct fs_disk_context_s *disk_context,
			      struct fs_handle_s *handle,
			      uint8_t *buffer,
			      uint32_t size)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  struct vfat_handle_s *hndl = handle->pv;

}
