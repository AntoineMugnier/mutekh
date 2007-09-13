#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include "vfat.h"
#include "vfat_private.h"

VFS_EXPORT error_t vfat_get_root_info(struct fs_disk_context_s *disk_context,
				      struct fs_entity_s *entity)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  uint8_t cluster[ctx->bytes_per_cluster];

  if (vfat_read_cluster(ctx, ctx->rootdir_first_cluster, cluster))
    return FS_ERROR_BADBLK;

  struct vfat_DirEntry_s *dirent = (struct vfat_DirEntry_s *)cluster;

  if (dirent->DIR_Attr & VFAT_ATTR_VOLUME_ID)
    {
      entity->name = strdup("/");
      entity->size = 0;
      entity->flags |= FS_ENT_DIRECTORY;

      struct vfat_entity_s *e = mem_alloc(sizeof(*e), MEM_SCOPE_SYS);

      if (e != NULL)
	{
	  e->clus_idx = ctx->rootdir_first_cluster;
	  entity->pv = (void *)e;

	  return FS_OK;
	}
      return FS_ERROR_ALLOC;
    }
  return FS_ERROR;
}

struct vfat_entity_s *vfat_entity_alloc_vfat_info(struct fs_entity_s *entity)
{
  struct vfat_entity_s *e = mem_alloc(sizeof(*e), MEM_SCOPE_SYS);

  return (entity->pv = (void *)e);
}

error_t vfat_setup_entity(struct fs_entity_s *entity,
			  struct vfat_DirEntry_s *dirent,
			  char *name)
{
  entity->name = strdup(name);

  ((struct vfat_entity_s *)(entity->pv))->clus_idx =
    (endian_le16_na_load(&dirent->DIR_FstClusHI) << 16) |
    endian_le16_na_load(&dirent->DIR_FstClusLO);

  if (dirent->DIR_Attr & VFAT_ATTR_DIRECTORY)
    {
      entity->size = 0;
      entity->flags |= FS_ENT_DIRECTORY;
    }
  else
    {
      entity->size = endian_le32_na_load(&dirent->DIR_FileSize);
      entity->flags |= FS_ENT_FILE;
    }
  return FS_OK;
}

error_t vfat_destroy_entity(struct fs_entity_s *entity)
{
  mem_free(entity->pv);
  mem_free(entity->name);

  return 0;
}

error_t vfat_init_entity(struct fs_entity_s *entity)
{
  if (vfat_entity_alloc_vfat_info(entity) == NULL)
    return -1;
  return 0;
}

VFS_EXPORT error_t vfat_get_entity_info(struct fs_disk_context_s *disk_context,
					struct fs_entity_s *parent,
					struct fs_entity_s *entity,
					char *name)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  struct vfat_DirEntry_s *dirent;
  char filename[256];
  struct vfat_handle_s hdir;
  uint8_t cluster[ctx->bytes_per_cluster];

  hdir.clus_idx = ((struct vfat_entity_s *)(parent->pv))->clus_idx;
  hdir.clus_offset = 0;
  hdir.next_cluster = hdir.clus_idx;

#ifdef CONFIG_DRIVER_VFAT_BLOCK_CACHE
  hdir.cluster = cluster;
#endif

  if (vfat_get_next_entry(ctx, &hdir, &dirent, filename, cluster) < 0)
    return FS_ERROR;

  do
    {
      if (!strcasecmp(filename, name))
	goto entry_found;
    }
  while(vfat_get_next_entry(ctx, &hdir, &dirent, filename, cluster) >= 0);
  return FS_ERROR;

 entry_found:
  if (vfat_entity_alloc_vfat_info(entity) == NULL)
    return FS_ERROR_ALLOC;

  return (vfat_setup_entity(entity, dirent, filename));
}
