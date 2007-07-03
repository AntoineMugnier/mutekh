#include <stdlib.h>
#include <string.h>
#include <hexo/endian.h>
#include "vfat.h"
#include "vfat_private.h"

static inline void vfat_getshortname(char *from, char *to)
{
  char *p = to;
  char *q = from;
  uint8_t d = 0;

  while (d++ < 8 && *q != ' ')
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

static inline error_t vfat_getnextdirentry(struct vfat_disk_context_s *ctx,
					   struct vfat_handle_s *hdir,
					   struct vfat_DirEntry_s **dirent,
					   uint8_t *cluster)
{
#ifdef CONFIG_DRIVER_VFAT_BLOCK_CACHE
      /*
	loads next cluster
	(either because it's the first call of get_next_entry or
	we've reached the last record in the current cluster and
	there are other remaining entries in the directory)
      */
      if (hdir->clus_offset >= ctx->bytes_per_cluster)
	{
	  if (vfat_read_cluster(ctx, hdir->next_cluster, cluster))
	    return -1;
#else /* CONFIG_DRIVER_VFAT_BLOCK_CACHE */
      if (vfat_read_cluster(ctx, hdir->next_cluster, cluster))
	return -1;
      if (hdir->clus_offset >= ctx->bytes_per_cluster)
	{
#endif
	  hdir->next_cluster = vfat_query_fat(ctx, hdir->next_cluster);
	  hdir->clus_offset = 0;
	}
      *dirent = (struct vfat_DirEntry_s *)(&cluster[hdir->clus_offset]);
      hdir->clus_offset += 0x20;
  return 0;
}

static inline uint16_t vfat_accum_lfn(struct vfat_LongDirEntry_s *ldirent,
				      char *filename,
				      uint8_t idx)
{
#define VFAT_LFN_GETCHAR(part, number)					\
  c = endian_le16_na_load(&ldirent->LDIR_Name##part[number]) >> 4;	\
  filename[255 - idx++] = c;						\
  if (c == 0)								\
    return idx;

  uint16_t c;

  VFAT_LFN_GETCHAR(3, 1);
  VFAT_LFN_GETCHAR(3, 0);

  VFAT_LFN_GETCHAR(2, 5);
  VFAT_LFN_GETCHAR(2, 4);
  VFAT_LFN_GETCHAR(2, 3);
  VFAT_LFN_GETCHAR(2, 2);
  VFAT_LFN_GETCHAR(2, 1);
  VFAT_LFN_GETCHAR(2, 0);

  VFAT_LFN_GETCHAR(1, 4);
  VFAT_LFN_GETCHAR(1, 3);
  VFAT_LFN_GETCHAR(1, 2);
  VFAT_LFN_GETCHAR(1, 1);
  VFAT_LFN_GETCHAR(1, 0);

  return idx;
}

error_t vfat_get_next_entry(struct vfat_disk_context_s *ctx,
			    struct vfat_handle_s *hdir,
			    struct vfat_DirEntry_s **dirent,
			    char *filename,
			    uint8_t *cluster)
{
  uint16_t lfn = 0;

  while (1)
    {
      if (vfat_getnextdirentry(ctx, hdir, dirent, cluster))
	return -1;
      if ((*dirent)->DIR_Name[0] == 0x00) /* Last record, goodbye */
	return -2;
      if ((*dirent)->DIR_Name[0] == 0xE5) /* Empty record, skip */
	continue;
      if ((*dirent)->DIR_Attr & 0x0F) /* Long File Name, parse, iterate */
	{
	  /* FIXME */
	  continue;
	  /*
	    vfat_accum_lfn simply does not work
	  */
	  if (lfn < 255 - 13)
	    lfn += vfat_accum_lfn((struct vfat_LongDirEntry_s *)(*dirent),
				  filename,
				  lfn);
	  continue;
	}
      if (lfn) /* first short entry after LFN records */
	{
	  memmove(filename + (255 - lfn), filename, lfn);
	  return 0;
	}
      else /* single short entry */
	{
	  if((*dirent)->DIR_Name[0] == '.') /* invalid ./.. entry */
	    continue;
	  vfat_getshortname((*dirent)->DIR_Name, filename);
	  return 0;
	}
    }
}

VFS_EXPORT error_t vfat_find_first_file(struct fs_disk_context_s *disk_context,
					struct fs_handle_s *parent,
					struct fs_entity_s *entity,
					char *filename)
{
  struct vfat_handle_s *hdir = (struct vfat_handle_s *)(parent->pv);

  hdir->clus_idx = ((struct vfat_entity_s *)(parent->ent->pv))->clus_idx;
  hdir->clus_offset = 0;
  hdir->next_cluster = hdir->clus_idx;

  return (vfat_find_next_file(disk_context, 
			      parent,
			      entity,
			      filename));
}

static inline error_t vfat_setup_fake_entity(struct fs_entity_s *entity,
					     struct vfat_DirEntry_s *dirent,
					     char *name)
{
  entity->name = name;

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
  return 0;
}

VFS_EXPORT error_t vfat_find_next_file(struct fs_disk_context_s *disk_context,
				       struct fs_handle_s *parent,
				       struct fs_entity_s *entity,
				       char *filename)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  struct vfat_DirEntry_s *dirent;
  char cluster[ctx->bytes_per_cluster];

  if (vfat_get_next_entry(ctx,
			  (struct vfat_handle_s *)(parent->pv),
			  &dirent,
			  filename,
			  cluster) < 0)
    return 0;

  if (entity != NULL)
    {
      vfat_setup_fake_entity(entity, dirent, filename);
    }
  return 1;
}
