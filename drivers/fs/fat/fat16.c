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

#include <fat-access.h>

struct fat_bpb_s
{
	uint8_t jmpboot[3];
	uint8_t oemname[8];
	uint16_t byte_per_sector;
	uint8_t sector_per_cluster;
	uint16_t reserved_sect_count;
	uint8_t fat_count;
	uint16_t root_dirent_count;
	uint16_t total_sector_count16;
	uint8_t media;
	uint16_t fat_size;
	uint8_t drive_info[8];
	uint32_t total_sector_count32;
	/* Fat 12/16 specific from here */
	uint8_t drive_number;
	uint8_t nt_reserved;
	uint8_t boot_sig;
	uint8_t volume_id[4];
	uint8_t volume_label[11];
	uint8_t volume_type[8];
} __attribute__((packed));

#define CLUSTER_END ((uint16_t)0xffff)

struct fat_s;
struct fat_extent_cache_s;

CONTAINER_TYPE(fat_extent, CLIST, struct fat_extent_s
{
    CONTAINER_ENTRY_TYPE(CLIST) list_entry;
    uint16_t cluster_contig_no; // index from start of file
	uint16_t cluster_contig_first; // index in fat
	uint16_t cluster_contig_count; // count in fat entries
	uint16_t cluster_next; // next fat index where it may be contiguous
    struct fat_extent_cache_s *cache;
}, list_entry);

CONTAINER_FUNC(fat_extent, CLIST, static inline, fat_extent, list_entry);

CONTAINER_TYPE(fat_extent_cache, CLIST,
struct fat_extent_cache_s
{
    CONTAINER_ENTRY_TYPE(CLIST) list_entry;
    struct fat_s *fat;
    fat_extent_root_t files;
    struct rwlock_s lock;
    uint32_t file_size;
    uint16_t first_cluster;
}, list_entry);

CONTAINER_FUNC(fat_extent_cache, CLIST, static inline, fat_extent_cache, list_entry);

/*
  The fat context.

  TODO: proper locking
*/
struct fat_s
{
	struct device_s *dev;
	uint32_t total_sector_count;
	uint32_t fat_secsize;
	uint32_t cluster0_sector; /* == first_data_sector - (2 << sect_per_clust_pow2) */
	uint16_t root_dir_secsize;
	uint16_t reserved_sect_count;
	uint16_t first_probable_free_cluster;
	uint8_t sect_size_pow2;
	uint8_t sect_per_clust_pow2;
	uint8_t fat_count;

	// This block is probably not right here, but we must guarantee
	// not to overflow caller's stack...

    bool_t tmp_block_dirty;
	// having a uint32_t here makes tmp_block aligned
    uint32_t tmp_block_no;
	uint8_t tmp_block[0];
};

// Loads a given sector in the buffer
static error_t fat16_load_sector(struct fat_s *state, dev_block_lba_t lba)
{
	error_t err;
	uint8_t *blocks[1] = {state->tmp_block};

    if ( state->tmp_block_no == lba )
        return 0;

    if ( state->tmp_block_dirty ) {
        err = dev_block_wait_write(state->dev, blocks, state->tmp_block_no, 1);
        if ( err )
            return err;
        state->tmp_block_dirty = 0;
        state->tmp_block_no = CLUSTER_END;
    }

	err = dev_block_wait_read(state->dev, blocks, lba, 1);
	if ( err )
		return err;
    state->tmp_block_no == lba;
	return 0;
}

// loads the bpb and parses it. This fuctnion is checking properties
// and yields error if they are not good
static error_t parse_bpb(struct fat_s *state)
{
	struct fat_bpb_s *bpb = (struct fat_bpb_s *)state->tmp_block;
	struct dev_block_params_s *params = dev_block_getparams(state->dev);

	err = fat16_load_sector(state, 0);
	if ( err )
		return err;

	if ( (state->tmp_block[510] != 0x55)
		 || (state->tmp_block[511] != 0xaa) ) {
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
		bpb->reserved_sect_count +
		state->fat_secsize * state->fat_count +
		state->root_dir_secsize -
		(2 << state->sect_per_clust_pow2);
    state->first_probable_free_cluster = 2;

	return 0;
}

// thinking of fat32, this has to be in a func
static inline
bool_t fat_entry16_is_free(uint16_t entry)
{
	return entry == 0;
}

static FAT_BACKEND_OPEN(fat16_backend_open)
{
	error_t err = -ENOMEM;
	struct dev_block_params_s *params = dev_block_getparams(dev);
	struct fat_s *state = mem_alloc(sizeof(struct fat_s)+params->blk_size,
									mem_scope_sys);
	if ( state == NULL )
		goto cant_alloc;

	state->dev = dev;

	err = parse_bpb(state);
	if ( err )
		goto cant_open;

	*fat = state;
	return 0
  cant_open:
	mem_free(state);
  cant_alloc:
	return err;
}

static FAT_BACKEND_SYNC(fat16_backend_sync)
{
	return 0;
}

static FAT_BACKEND_CLOSE(fat16_backend_close)
{
	fat16_backend_sync(fat);
	mem_free(fat);
	return 0;
}

static inline void fat16_cut_fat_cluster_no(
    struct fat_s *fat,
    uint16_t cluster_no,
    size_t *fat_block_no,
    size_t *fat_offset)
{
    *fat_block_no = cluster_no >> (fat->sect_size_pow2 - 1);
    *fat_offset = cluster_no & ((1 << (fat->sect_size_pow2 - 1)) - 1);
}

static uint16_t fat16_get_next_pointer(struct fat_s *fat, uint16_t cluster_no)
{
    uint16_t fat_sector;
    uint16_t fat_offset;
    fat16_cut_fat_cluster_no(fat, cluster_no, &fat_sector, &fat_offset);

    if ( fat_sector >= fat->fat_secsize )
        return CLUSTER_END;

    fat16_load_sector(fat, fat_sector);
    uint16_t *data = (uint16_t*)fat->tmp_block;

    return data[fat_offset];
}

static uint16_t fat16_get_free_free_cluster(struct fat_s *fat)
{
    size_t i;

    for ( i = fat->first_probable_free_cluster;
          i < fat->total_sector_count;
          ++i ) {
        uint16_t next_pointer = fat16_get_next_pointer(fat, i);
        if ( next_pointer == 0 ) {
            fat->first_probable_free_cluster = i + 1;
            return i;
        }
    }
    return CLUSTER_END;
}

static void fat16_set_fat(struct fat_s *fat, uint16_t cluster_no, uint16_t next_cluster_no)
{
    uint16_t fat_sector;
    uint16_t fat_offset;
    fat16_cut_fat_cluster_no(fat, cluster_no, &fat_sector, &fat_offset);
    uint16_t *data = (uint16_t*)fat->tmp_block;

    data[fat_offset] = next_cluster_no;
}

static FAT_EXTENT_GET_NEW(fat16_extent_get_new)
{
    struct fat_extent_cache_s *extent = fat16_extent_new();
    
}

static FAT_EXTENT_GET_AT(fat16_extent_get_at)
{
}

static FAT_EXTENT_GET_SIZE(fat16_extent_get_size)
{
	return extent->file_size;
}

static FAT_EXTENT_RESIZE(fat16_extent_resize)
{
    if ( extent->new_cluster_count 
}

static FAT_EXTENT_RELEASE(fat16_extent_release)
{

}

static FAT_EXTENT_BLOCK2LBA(fat16_extent_block2lba)
{

}

const fat_access_ops_s fat16_access =
{
	.backend_open = fat16_backend_open,
	.backend_sync = fat16_backend_sync,
	.backend_close = fat16_backend_close,
	.extent_get_new = fat16_extent_get_new,
	.extent_get_at = fat16_extent_get_at,
	.extent_get_size = fat16_extent_get_size,
	.extent_resize = fat16_extent_resize,
	.extent_release = fat16_extent_release,
	.extent_block2lba = fat16_extent_block2lba,
};
