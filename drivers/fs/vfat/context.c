#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include <hexo/endian.h>
#include "vfat.h"
#include "vfat_private.h"

error_t vfat_pv_build_fat_map(struct vfat_disk_context_s *ctx,
			      uint_fast32_t map_sz,
			      uint32_t *fat_buff)
{
  /*
    uint_fast32_t nb_blk = ctx->fat_blk_count;
    uint_fast32_t blk_idx = 0;

    while (nb_blk--)
    {
    if (vfat_read_blk(ctx, ctx->fat_begin_lba + blk_idx, (uint8_t *)fat_buff))
    return -1;

    for (i = 0; i < ctx->bytes_per_sector / 4; i += 8)
    {
    (fat_buff[i] & 0x0FFFFFFF);

    }
    }
  */
}

static inline error_t vfat_context_init(struct vfat_disk_context_s *ctx)
{
  size_t blk_sz = dev_block_getparams(ctx->fs.device)->blk_size;
  struct vfat_bpb_s *bpb;
  struct dev_block_rq_s rq;
  uint8_t *data[1];
  uint8_t d[blk_sz];

  data[0] = d;
  rq.lba = 0;
  rq.count = 1;
  rq.data = data;

  if (dev_block_wait_read(ctx->fs.device, &rq))
    {
      printf("VFAT drv error: IO error, couldn't read bpb for device.\n");
      return -1;
    }

  bpb = (struct vfat_bpb_s *)d;

  /* CHECKME/FIXME
     We will take for granted that the physical block size of the
     device must be equivalent to the one provided by the BPB.
  */
  if (blk_sz != (ctx->bytes_per_sector = endian_le16_na_load(&bpb->BPB_BytsPerSec)))
    {
      printf("VFAT drv error: bpb/device block size mismatch.\n");
      return -1;
    }

  ctx->fat_begin_lba = endian_le16_na_load(&bpb->BPB_RsvdSecCnt);
  ctx->fat_blk_count = endian_le32_na_load(&bpb->BPB_FATSz32);
  ctx->cluster_begin_lba = ctx->fat_begin_lba +
    (bpb->BPB_NumFATs * endian_le32_na_load(&bpb->BPB_FATSz32));
  ctx->sectors_per_cluster = bpb->BPB_SecPerClus;
  ctx->rootdir_first_cluster = endian_le32_na_load(&bpb->BPB_RootClus);
  ctx->bytes_per_cluster = ctx->bytes_per_sector * ctx->sectors_per_cluster;

  uint_fast32_t map_sz = ctx->fat_blk_count  * ctx->bytes_per_sector / 32;
  if((ctx->fat_map = mem_alloc(map_sz, MEM_SCOPE_SYS)) == NULL)
    {
      printf("VFAT drv error: FAT write cache allocation failed (asked for %d bytes).\n", map_sz);
      return -1;
    }
  
  vfat_pv_build_fat_map(ctx, map_sz, (uint32_t *)d);

  return 0;
}

VFS_EXPORT struct fs_disk_context_s *vfat_context_create(struct device_s *device)
{
  struct vfat_disk_context_s *ctx;

  if ((ctx = mem_alloc(sizeof(*ctx), MEM_SCOPE_SYS)) == NULL)
    return NULL;

  ctx->fs.device = device;

  if (vfat_context_init(ctx))
    {
      mem_free(ctx);
      return NULL;
    }
  return ((struct fs_disk_context_s *)ctx);
}

VFS_EXPORT error_t vfat_context_destroy(struct fs_disk_context_s *disk_context)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;

  assert(ctx != NULL);

  mem_free(ctx);
  return FS_OK;
}
