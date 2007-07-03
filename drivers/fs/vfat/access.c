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

uint32_t vfat_query_fat(struct vfat_disk_context_s *ctx,
			uint32_t clus_idx)
{
  uint8_t buffer[ctx->bytes_per_cluster];
  uint32_t clus_sh = ffsl(ctx->bytes_per_cluster) - 1;
  uint32_t fat_idx = clus_idx >> clus_sh;
  uint32_t fat_off = clus_idx & (clus_sh - 1);

  if (vfat_read_cluster(ctx, fat_idx, buffer))
    return 0;

  return (*((uint32_t *)(buffer + fat_off)) & 0x0FFFFFFF);
}
