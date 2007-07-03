#ifndef __VFAT_PRIVATE_H__
#define __VFAT_PRIVATE_H__
#include "vfat.h"

/* access.c */
error_t vfat_read_cluster(struct vfat_disk_context_s *ctx,
			  size_t clus_offset,
			  uint8_t *clus_buffer);

uint32_t vfat_query_fat(struct vfat_disk_context_s *ctx,
			uint32_t clus_idx);

/* entity.c */

struct vfat_entity_s *vfat_entity_alloc_vfat_info(struct fs_entity_s *entity);

error_t vfat_setup_entity(struct fs_entity_s *entity,
			  struct vfat_DirEntry_s *dirent,
			  char *name);

error_t vfat_destroy_entity(struct fs_entity_s *entity);

/* directory.c */
error_t vfat_get_next_entry(struct vfat_disk_context_s *ctx,
			    struct vfat_handle_s *hdir,
			    struct vfat_DirEntry_s **dirent,
			    char *filename,
			    uint8_t *cluster);

#endif
