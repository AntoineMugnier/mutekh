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

// TODO use GPCT, use a custom allocator
struct fat_extent_chain_s
{
	uint16_t base;
	uint16_t size;
	struct fat_extent_chain_s *next;
}

// TODO use GPCT, use a custom allocator
// TODO a fast way to seek in the list (hash ?) in case the drive is too fragmented
struct fat_extent_s
{
	struct fat_extent_chain_s *first;
	uint16_t size; // in clusters == sum(chain.size foreach chain)
}


/*
  The fat context.
  
  TODO: add a list of currently-known extents, indexed by their first
  cluster index.

  TODO: proper locking
*/
struct fat_s
{
	struct device_s *dev;
	struct fat_extent_s free;
	uint32_t total_sector_count;
	uint32_t fat_secsize;
	uint32_t cluster0_sector; /* == first_data_sector - (2 << sect_per_clust_pow2) */
	uint16_t root_dir_secsize;
	uint16_t reserved_sect_count;
	uint8_t sect_size_pow2;
	uint8_t sect_per_clust_pow2;
	uint8_t fat_count;
	uint8_t __pad; // make tmp_block aligned

	// This block is probably not right here, but we must guarantee
	// not to overflow caller's stack...
	uint8_t tmp_block[0];
};

// Loads a given sector in the buffer
static error_t fat16_load_sector(struct fat_s *state, dev_block_lba_t lba)
{
	error_t err;
	uint8_t *blocks[1] = {state->tmp_block};

	err = dev_block_wait_read(state->dev, blocks, lba, 1);
	if ( err )
		return err;
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

	return 0;
}

// thinking of fat32, this has to be in a func
static inline
bool_t fat_entry16_is_free(uint16_t entry)
{
	return entry == 0;
}

// retrieves the freelist from the FAT.
static
error_t parse_freelist(struct fat_s *state)
{
	size_t i, j;
	error_t err;
	dev_block_lba_t cluster;
	dev_block_lba_t fat_sector = state->reserved_sect_count;
	const dev_block_lba_t fat_end = state->reserved_sect_count + state->fat_secsize;
	const dev_block_lba_t cluster_mask = (1 << (state->sect_size_pow2 - 1)) - 1;
	uint16_t *fat_table = (uint16_t *)state->tmp_block;

	struct fat_extent_chain_s * extent = NULL;

	state->free_cluster_count = 0;

	err = fat16_load_sector(state, state->reserved_sect_count);
	if ( err )
		return err;

	for ( cluster = 2; cluster < state->total_cluster_count; ++cluster ) {
		size_t offset = cluster & cluster_mask;

		if ( offset == 0 ) {
			err = fat16_load_sector(state, cluster >> (state->sect_size_pow2 - 1));
			if ( err )
				return err;
		}

		if ( fat_entry16_is_free(fat_table[offset]) ) {
			// append in freelist, but merge contiguous ones
		}
	}
	
	return 0;
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

	err = parse_freelist(state);
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


static FAT_EXTENT_GET_NEW(fat16_extent_get_new)
{
	if ( size > fat->free_cluster_count )
		return -ENOMEM;

	// pop size clusters from free list.
}

static FAT_EXTENT_GET_AT(fat16_extent_get_at)
{
}

static FAT_EXTENT_GET_SIZE(fat16_extent_get_size)
{
	return extent->size;
}

static FAT_EXTENT_RESIZE(fat16_extent_resize)
{

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
