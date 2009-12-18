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

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <device/block.h>
#include <device/driver.h>
#include <device/device.h>
#include <vfs/types.h>
#include <vfs/file.h>
#include <vfs/fs.h>

#include "fat-sector-cache.h"
#include "fat.h"

#define FAT_16
#include "fat-defs.h"
#include "fat-private.h"

VFS_FILE_CLOSE(fat_file_close);
VFS_FILE_READ(fat_dir_read);
VFS_FILE_READ(fat_file_read);
VFS_FILE_WRITE(fat_file_write);
VFS_FILE_SEEK(fat_file_seek);

static const struct vfs_fs_ops_s fat16_ops;

static const struct fat_ops_s fat16_fat_ops;

// loads the bpb and parses it. This fuctnion is checking properties
// and yields error if they are not good
static error_t fat16_parse_bpb(struct fat_s *state, struct fat_tmp_sector_s *sector)
{
	struct fat_bpb_s *bpb = (struct fat_bpb_s *)sector->data;
	const struct dev_block_params_s *params = dev_block_getparams(state->dev);

	error_t err = fat_sector_lock_and_load(sector, state->dev, 0);
	if ( err )
		return err;

/*     printk("FAT16 BPB:\n"); */
/*     hexdumpk(sector->data, params->blk_size); */

	if ( (sector->data[510] != 0x55)
		 || (sector->data[511] != 0xaa) ) {
		printk("FAT Error: 55AA marker not found\n");
		return -ENOENT;
	}

	if ( bpb->byte_per_sector != params->blk_size ) {
		printk("FAT Error: device sector size (%d) does not match FAT's (%d)\n",
			   params->blk_size, bpb->byte_per_sector);
		return -EINVAL;
	}

	state->total_sector_count = bpb->total_sector_count16
		? bpb->total_sector_count16
		: bpb->total_sector_count32;

	if ( state->total_sector_count < params->blk_count ) {
		printk("FAT Error: device size (%d blocks) is less than FAT's (%d blocks)\n",
			   params->blk_count, state->total_sector_count);
		return -EINVAL;
	}
	
	state->fat_secsize = bpb->fat_size;
	state->sect_size_pow2 = __builtin_ctz(bpb->byte_per_sector);
	state->sect_per_clust_pow2 = __builtin_ctz(bpb->sector_per_cluster);
	state->root_dir_secsize = bpb->root_dirent_count >> (state->sect_size_pow2 - 5);
	state->fat_count = bpb->fat_count;
	state->reserved_sect_count = bpb->reserved_sect_count;
	state->cluster0_sector =
		bpb->reserved_sect_count
		+ state->fat_secsize * state->fat_count
		+ state->root_dir_secsize
		- (2 << state->sect_per_clust_pow2);
    state->first_probable_free_cluster = 2;

    printk("FAT16 decoded BPB:\n");
    printk(" total_sector_count:  %d\n", state->total_sector_count);
    printk(" fat_secsize:         %d\n", state->fat_secsize);
    printk(" cluster0_sector:     %d\n", state->cluster0_sector);
    printk(" root_dir_secsize:    %d\n", state->root_dir_secsize);
    printk(" reserved_sect_count: %d\n", state->reserved_sect_count);
    printk(" first_probable_free_cluster: %d\n", state->first_probable_free_cluster);
    printk(" sect_size_pow2:      %d\n", state->sect_size_pow2);
    printk(" sect_per_clust_pow2: %d\n", state->sect_per_clust_pow2);
    printk(" fat_count:           %d\n", state->fat_count);

    fat_sector_lock_release(sector);
	return 0;
}

static inline
bool_t fat_entry16_is_free(cluster_t entry)
{
	return (entry & CLUSTER_MASK) == 0;
}

error_t fat16_open(struct device_s *dev, struct vfs_fs_s **fs)
{
	error_t err = -ENOMEM;
	const struct dev_block_params_s *params = dev_block_getparams(dev);
    struct fat_s *fat = mem_alloc(
        sizeof(struct fat_s)
        +sizeof(struct fat_tmp_sector_s)
        +params->blk_size,
        mem_scope_sys);

	if ( fat == NULL )
		goto cant_alloc;

    struct vfs_fs_s *mnt = vfs_fs_new(&fat->fs);

    fat->sector = (struct fat_tmp_sector_s*)(fat+1);
    fat->sector->lba = (dev_block_lba_t)-1;
    fat->sector->dirty = 0;
    semaphore_init(&fat->sector->semaphore, 1);
	fat->dev = dev;

	err = fat16_parse_bpb(fat, fat->sector);
	if ( err )
		goto cant_open;

    printk("fat16: opening new fat16 volume\n");

	atomic_set(&mnt->ref, 0);
	mnt->ops = &fat16_ops;
	fat->ops = &fat16_fat_ops;
	mnt->old_node = NULL;
    fat_node_pool_init(&fat->nodes);

    struct fs_node_s *root = fat_node_new(
        NULL, fat,
        0, /* fixed for fat16 */
        fat->root_dir_secsize << fat->sect_size_pow2,
        VFS_NODE_DIR);
	if ( root == NULL )
		goto cant_open;

	mnt->root = root;

	*fs = mnt;
	return 0;

  cant_open:
	mem_free(mnt);
  cant_alloc:
	return err;
}

static inline error_t fat16_flush(struct fat_s *fat)
{
	return fat_sector_flush(fat->sector, fat->dev);
}

VFS_FS_CAN_UNMOUNT(fat16_can_unmount)
{
    // TODO copy first FAT on others

    return 0;
}




static inline void fat16_cut_fat_cluster_no(
    struct fat_s *fat,
    cluster_t cluster_no,
    cluster_t *fat_block_no,
    cluster_t *fat_offset)
{
    *fat_block_no = (cluster_no >> (fat->sect_size_pow2 - 1)) + fat->reserved_sect_count;
    *fat_offset = cluster_no & ((1 << (fat->sect_size_pow2 - 1)) - 1);
}

static common_cluster_t fat16_entry_get(
    struct fat_s *fat,
    common_cluster_t cluster_no)
{
    uint16_t fat_sector;
    uint16_t fat_offset;
    fat16_cut_fat_cluster_no(fat, cluster_no, &fat_sector, &fat_offset);

    if ( fat_sector >= fat->fat_secsize )
        return CLUSTER_END;

    error_t err = fat_sector_lock_and_load(fat->sector, fat->dev, fat_sector);
    if ( err )
        return err;
    uint16_t *data = (uint16_t*)fat->sector->data;

    vfs_printk("<%s(%d) = %d>", __FUNCTION__, cluster_no, data[fat_offset]);

    uint16_t ret = data[fat_offset];
    fat_sector_lock_release(fat->sector);
    return ret;
}

static common_cluster_t fat16_entry_find_free(
    struct fat_s *fat)
{
    size_t i;

    for ( i = fat->first_probable_free_cluster;
          i < fat->total_sector_count;
          ++i ) {
        cluster_t next_pointer = fat16_entry_get(fat, i);
        if ( next_pointer == 0 ) {
            fat->first_probable_free_cluster = i + 1;
            return i;
        }
    }
    return CLUSTER_END;
}

static error_t fat16_entry_set(
    struct fat_s *fat,
    common_cluster_t cluster_no,
    common_cluster_t next_cluster_no)
{
    uint16_t fat_sector;
    uint16_t fat_offset;
    fat16_cut_fat_cluster_no(fat, cluster_no, &fat_sector, &fat_offset);

    error_t err = fat_sector_lock_and_load(fat->sector, fat->dev, fat_sector);
    if ( err )
        return err;

    uint16_t *data = (uint16_t*)fat->sector->data;
    data[fat_offset] = next_cluster_no;
    fat_sector_lock_release(fat->sector);
    return 0;
}

static bool_t fat16_entry_is_end(common_cluster_t cluster)
{
    return (uint16_t)((cluster | 0x7) + 1) == 0;
}

static const struct vfs_fs_ops_s fat16_ops =
{
    .node_open = fat_node_open,
    .lookup = fat_lookup,
    .create = fat_create,
    .link = fat_link,
    .move = fat_move,
    .unlink = fat_unlink,
    .stat = fat_stat,
    .can_unmount = fat16_can_unmount,
    .node_refdrop = fat_node_refdrop,
    .node_refnew = fat_node_refnew,
};

static const struct fat_ops_s fat16_fat_ops =
{
    .entry_get = fat16_entry_get,
    .entry_set = fat16_entry_set,
    .entry_find_free = fat16_entry_find_free,
    .entry_is_end = fat16_entry_is_end,
};
