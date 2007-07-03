#include <hexo/alloc.h>
#include "vfat.h"

error_t vfat_init_handle(struct fs_disk_context_s *disk_context,
			 struct fs_handle_s *handle,
			 struct fs_entity_s *entity)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  struct vfat_handle_s *h;

  h = mem_alloc(sizeof(*h), MEM_SCOPE_SYS);
  if (h != NULL)
    {
      h->clus_idx = ((struct vfat_entity_s *)(entity->pv))->clus_idx;
      h->clus_offset = ctx->bytes_per_cluster;
      h->next_cluster = h->clus_idx;

#ifdef CONFIG_DRIVER_VFAT_BLOCK_CACHE
      h->cluster = mem_alloc(ctx->bytes_per_cluster, MEM_SCOPE_SYS);
      if (h->cluster == NULL)
	{
	  mem_free(h);
	  return -5;
	}
#endif
      handle->pv = (void *)h;
      return 0;
    }
  return -4;
}

void vfat_release_handle(struct fs_handle_s *handle)
{
#ifdef CONFIG_DRIVER_VFAT_BLOCK_CACHE
  mem_free(((struct vfat_handle_s *)(handle->pv))->cluster);
#endif
  mem_free(handle->pv);
}
