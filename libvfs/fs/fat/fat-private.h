/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.
    
  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
    
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009,2014
*/

#ifndef _FAT_PRIVATE_H_
#define _FAT_PRIVATE_H_

#include <hexo/types.h>
#include <vfs/node.h>
#include <vfs/fs.h>
#include <vfs/file.h>

#include <mutek/semaphore.h>

#include <gct/container_avl_p.h>
#include <gct/container_clist.h>

#include "fat-defs.h"

struct fat_s;
struct fat_node_s;
struct fat_ops_s;

struct fat_file_s;

#define GCT_CONTAINER_LOCK_fat_file_list HEXO_LOCK_IRQ
#define GCT_CONTAINER_ALGO_fat_file_list CLIST

struct fat_file_s
{
    struct vfs_file_s file;     /* keep first field */

    common_cluster_t cluster_index;      // index from start of *file*
    common_cluster_t zone_start; // index of first contiguous block in fat
    common_cluster_t zone_end;   // index of last contiguous block in fat + 1

    GCT_CONTAINER_ENTRY(fat_file_list, list_entry);
};

/* This list is used to keep track of all existing file objects for a node. */
GCT_CONTAINER_TYPES(fat_file_list, struct fat_file_s *, list_entry);
GCT_CONTAINER_KEY_TYPES(fat_file_list, PTR, SCALAR, cluster_index);
GCT_CONTAINER_KEY_FCNS(fat_file_list, ASC, static inline, fat_file_list, cluster_index,
                   init, destroy, push, remove, isempty);

struct fat_file_s *fat_file_create(struct fat_node_s *node,
                                   enum vfs_open_flags_e flags);

static inline struct fat_file_s *fat_file_refinc(struct fat_file_s *file)
{
    return (struct fat_file_s *)vfs_file_refinc(&file->file);
}

static inline bool_t fat_file_refdec(struct fat_file_s *file)
{
    return vfs_file_refdec(&file->file);
}

#define GCT_CONTAINER_LOCK_fat_node_pool HEXO_LOCK_IRQ
#define GCT_CONTAINER_ALGO_fat_node_pool AVL_P

struct fat_node_s
{
    struct vfs_node_s node;       /* keep first field */

    fat_file_list_root_t files;
    struct semaphore_s lock;
    uint32_t file_size;
    common_cluster_t first_cluster;

    GCT_CONTAINER_ENTRY(fat_node_pool, hash_entry);
};

static inline struct fat_node_s *fat_node_refinc(struct fat_node_s *node)
{
    return (struct fat_node_s *)vfs_node_refinc(&node->node);
}

static inline bool_t fat_node_refdec(struct fat_node_s *node)
{
    return vfs_node_refdec(&node->node);
}

/* This map is used to find when a node already exist for a given cluster index. */
GCT_CONTAINER_TYPES(fat_node_pool, struct fat_node_s *, hash_entry);
GCT_CONTAINER_KEY_TYPES(fat_node_pool, PTR, SCALAR, first_cluster);
GCT_CONTAINER_KEY_FCNS(fat_node_pool, ASC, static inline, fat_node_pool, first_cluster,
                       init, destroy, push, lookup, remove);

struct fat_node_s *fat_node_create(struct fat_s *fat,
                                   common_cluster_t first_cluster,
                                   size_t file_size,
                                   enum vfs_node_type_e type);

#define FAT_ENTRY_GET(x) common_cluster_t (x)                       \
    (struct fat_s *fat,                                             \
     common_cluster_t cluster)

#if defined(CONFIG_DRIVER_FS_FAT_RW)
# define FAT_ENTRY_SET(x) error_t (x)                                   \
    (struct fat_s *fat,                                             \
     common_cluster_t cluster, common_cluster_t next)

# define FAT_ENTRY_FIND_FREE(x) common_cluster_t (x)                 \
    (struct fat_s *fat)
#endif

struct fat_ops_s
{
    FAT_ENTRY_GET(*entry_get);
#if defined(CONFIG_DRIVER_FS_FAT_RW)
    FAT_ENTRY_SET(*entry_set);
    FAT_ENTRY_FIND_FREE(*entry_find_free);
#endif
};

enum fat_type_e
{
    FAT32 = 32,
    FAT16 = 16,
    FAT12 = 12,
};

struct fat_s
{
    struct vfs_fs_s fs;

    struct device_block_s *dev;
    const struct fat_ops_s *ops;
	common_cluster_t root_dir_secsize;
    // This holds a sector for fat16.
    common_cluster_t root_dir_base;
	common_cluster_t first_probable_free_cluster;
	sector_t total_sector_count;
	sector_t fat_secsize;
	sector_t cluster0_sector; /* == first_data_sector - (2 << sect_per_clust_pow2) */
	sector_t fat_sect0;
    fat_node_pool_root_t nodes;
    struct fat_tmp_sector_s *sector;
	uint8_t sect_size_pow2;
	uint8_t sect_per_clust_pow2;
	uint8_t fat_count;
	enum fat_type_e type;
};

static inline
struct fat_s *fs2fat(struct vfs_fs_s *fs)
{
    return (struct fat_s *)fs;
}

void fat_name_to_vfs(size_t dst_size, char *dst, const char *src);

union fat_dirent_u;

error_t fat_get_next_dirent(struct fat_file_s *ffile,
                            off_t *offset,
                            union fat_dirent_u *dirent,
                            char *name_83,
                            char *vfs_mangled_name);

ssize_t fat_data_read(
    struct fat_file_s *ffile,
    off_t offset,
    void *buffer, size_t size);

static inline bool_t fat_entry_is_end(common_cluster_t cluster)
{
#if defined(CONFIG_DRIVER_FS_FAT32)
    return ((cluster | 0xf0000007) + 1) == 0;
#else
    return ((cluster | 0x7) + 1) == 0;
#endif
}

VFS_FILE_READ(fat_dir_read);

void fat_str_to_lower(char *str, size_t size);

/*
  Common FAT FS operations
 */

VFS_FS_LOOKUP(fat_lookup);
VFS_FS_CREATE(fat_create);
VFS_FS_LINK(fat_link);
VFS_FS_MOVE(fat_move);
VFS_FS_UNLINK(fat_unlink);
VFS_FS_STAT(fat_stat);

VFS_FILE_READ(fat_dir_read);
VFS_FILE_READ(fat_file_read);
VFS_FILE_WRITE(fat_file_write);
VFS_FILE_SEEK(fat_file_seek);

#endif
