#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include <vfs/vfat/vfat.h>

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

static inline uint32_t vfat_get_next_cluster(struct vfat_disk_context_s *ctx,
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

static inline void vfat_get_short_name(char *from, char *to)
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

error_t vfat_get_root_info(struct fs_disk_context_s *disk_context,
			   struct fs_entity_s *file)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  struct vfat_entity_s *handle = (struct fs_entity_s *)file;
  uint8_t cluster[dir.cluster_sz];

  if (vfat_read_cluster(ctx, ctx->rootdir_first_cluster, &cluster))
    return -1;

  struct vfat_DirEntry_s *dirent = (struct vfat_DirEntry_s *)cluster;

  if (dirent->DIR_Attr & VFAT_ATTR_VOLUME_ID)
    {
      file->name = strdup("/");
      file->size = 0;
      file->flags |= FS_ENT_DIRECTORY;

      struct vfat_ent_dir_s *rd = mem_alloc(sizeof(*rd), MEM_SCOPE_SYS);

      if (rd != NULL)
	{
	  rd->clus_idx = ctx->rootdir_first_cluster;
	  file->e_pv = rd;
	  return 0;
	}
      return -3;
    }
  return -2;
}

static inline error_t vfat_get_next_entry(struct vfat_disk_context_s *ctx,
					  struct vfat_ent_dir_s *dir,
					  struct vfat_DirEntry_s **dirent,
					  uint8_t *cluster)
{
  if (dir->clus_offset >= dir->cluster_sz)
    {
      if (vfat_read_cluster(ctx, dir->next_cluster, cluster))
	return -1;
      dir->next_cluster = vfat_get_next_cluster(dsk_dev, dir->next_cluster);
      dir->clus_offset = 0;
    }

  *dirent = (struct vfat_DirEntry_s *)(&cluster[dir->clus_offset]);
  dir->clus_offset += 0x20;
  return 0;
}

static inline error_t vfat_ent_file_init(struct fs_entity_s *ent,
					 struct vfat_DirEntry_s *dirent,
					 char *filename)
{
  struct vfat_ent_file_s *file = mem_alloc(sizeof(*file), MEM_SCOPE_SYS);

  if (file != NULL)
    {
      file->clus_idx =
	(endian_le16_na_load(dirent->DIR_FstClusHI) << 16) |
	endian_le16_na_load(dirent->DIR_FstClusLO);
      ent->e_pv = file;

      ent->name = strdup(filename);
      ent->size = endian_le32_na_load(dirent->DIR_FileSize);
      ent->flags |= FS_ENT_FILE;

      return 0;
    }
  return -1;
}

static inline error_t vfat_ent_dir_init(struct fs_entity_s *ent,
					struct vfat_DirEntry_s *dirent,
					 char *filename)
{
  struct vfat_ent_file_s *dir = mem_alloc(sizeof(*dir), MEM_SCOPE_SYS);

  if (dir != NULL)
    {
      dir->clus_idx =
	(endian_le16_na_load(dirent->DIR_FstClusHI) << 16) |
	endian_le16_na_load(dirent->DIR_FstClusLO);
      ent->e_pv = dir;

      ent->name = strdup(filename);
      ent->size = 0;
      ent->flags |= FS_ENT_DIRECTORY;

      return 0;
    }
  return -1;
}


/* vfat_get_entity_info: retreive access informations for a given file

@param [IN]  disk_context	pointer to current driver context
@param [IN]  parent		pointer to parent's vfat_directory_s struct
@param [OUT] file		pointer to 
@param [IN]  fname		
*/

error_t vfat_get_entity_info(struct fs_disk_context_s *disk_context,
			     struct fs_entity_s *parent,
			     struct fs_entity_s *file,
			     char *fname)
{
  struct vfat_disk_context_s *ctx = (struct vfat_disk_context_s *)disk_context;
  struct vfat_ent_dir_s *pdir = (struct vfat_ent_dir_s *)(parent->e_pv);
  struct vfat_DirEntry_s *dirent;

  char filename[256];
  uint8_t cluster[ctx->bytes_per_cluster];

  /*
    clus_offset and next_cluster are set up in order
    to trigger a vfat_read_cluster() during the next
    call to vfat_get_next_entry();
  */
  pdir->clus_offset = ctx->bytes_per_cluster;
  pdir->next_cluster = pdir->clus_idx;

  do
    {
      if (vfat_get_next_entry(ctx, &dir, &dirent, cluster))
	return -1;
      if (dirent->DIR_Attr & VFAT_ATTR_LONG_NAME)
	{
	  //	  vfat_get_long_name(dirent->DIR_Name, filename, &idx);
	  continue;
	}
      else
	{
	  vfat_get_short_name(dirent->DIR_Name, filename);
	}
      if (!strcmp(filename, fname))
	goto ok;
    }
  while(dirent->DIR_Name[0] != 0);

  return -1;

 ok:
  if (dirent->DIR_Attr & FS_ENT_DIRECTORY)
    return (vfat_ent_dir_init(file, dirent, filename));
  else
    return (vfat_ent_file_init(file, dirent, filename));
}
