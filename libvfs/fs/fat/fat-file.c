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

#include <vfs/node.h>
#include <vfs/fs.h>
#include <vfs/file.h>

#include "fat-private.h"
#include "fat-defs.h"
#include "fat-sector-cache.h"

#include <hexo/types.h>
#include <mutek/mem_alloc.h>

struct fat_node_s *fat_node_create(struct fat_s *fat,
                                   common_cluster_t first_cluster,
                                   size_t file_size,
                                   enum vfs_node_type_e type)
{
    struct fat_node_s *node = mem_alloc(sizeof(*node), mem_scope_sys);
    if (!node)
        return NULL;

    if (vfs_node_init(&node->node, &fat->fs, type, NULL, 0)) {
        mem_free(node);
        return NULL;
    }

    node->first_cluster = first_cluster;
    node->file_size = file_size;

    vfs_printk("<%s: fc: %d size: %d %s>",
               __FUNCTION__, node->first_cluster,
               node->file_size, node->node.type == VFS_NODE_DIR ? "dir" : "file");

    fat_file_list_init(&node->files);
    semaphore_init(&node->lock, 1);

    fat_node_pool_push(&fat->nodes, node);

    return node;
}

VFS_FS_NODE_CLEANUP(fat_node_cleanup)
{
    struct fat_node_s *fnode = (void*)node;
    struct fat_s *fat = (void*)node->fs;

    fat_node_pool_remove(&fat->nodes, fnode);

    assert(fat_file_list_isempty(&fnode->files));
    fat_file_list_destroy(&fnode->files);

    semaphore_destroy(&fnode->lock);
}

static inline void dump_file(const struct fat_file_s *file)
{
    struct fat_node_s *fnode = (void*)file->file.node;

    vfs_printk("<fat file: first cluster: %d, size: %d, zone: @%d %d-%d>",
               fnode->first_cluster, fnode->file_size,
               file->cluster_index, file->zone_start, file->zone_end);
}

/** @this must be called with file->extent->lock taken. */
static
common_cluster_t fat_file_translate_cluster(
    struct fat_file_s *file,
    common_cluster_t cluster
    )
{
    common_cluster_t ret = cluster - file->cluster_index + file->zone_start;

    if ( ret < file->zone_end )
        return ret;

    return 0;
}

/** @this must be called with file->extent->lock wrlocked. */
static
common_cluster_t fat_file_get_cluster(
    struct fat_file_s *file,
    common_cluster_t cluster)
{
    struct fat_node_s *node = (void*)file->file.node;
    struct fat_s *fat = (void*)node->node.fs;
    common_cluster_t ret = 0;

    ret = fat_file_translate_cluster(file, cluster);
    if ( ret )
        goto found;

    /* First, get the most advanced in all other file for the same node */
    struct fat_file_s *best = NULL;
    GCT_FOREACH(fat_file_list, &node->files, item, {
            if ( (item->cluster_index < cluster) &&
                 ((best == NULL) || (item->cluster_index > best->cluster_index)) )
                best = item;
        });

    /* Copy the best one */
    if ( best == file ) {
        /* Noop */
    } else if ( best ) {
        file->cluster_index = best->cluster_index;
        file->zone_start = best->zone_start;
        file->zone_end = best->zone_end;
    } else {
        file->cluster_index = 0;
        file->zone_start = node->first_cluster;
        file->zone_end = node->first_cluster + 1;
    }

    /* Walk for the rest */
    while ( (ret = fat_file_translate_cluster(file, cluster)) == 0 ) {
        ret = fat->ops->entry_get(fat, file->zone_end - 1);
        if ( fat_entry_is_end(ret) ) {
            ret = 0;
            break;
        } else if ( ret == file->zone_end ) {
            file->zone_end += 1;
        } else {
            file->cluster_index += file->zone_end - file->zone_start;
            file->zone_start = ret;
            file->zone_end = ret + 1;
        }
    }

  found:
    return ret;
}

/**
   @this retrieves the maximal contiguous range of sectors from the
   current point in file (which MUST be aligned on a sector boundary).

   last_sector is the first sector after contiguous zone.
*/
static
error_t fat_file_get_biggest_sector_range(
    struct fat_file_s *ffile, off_t offset,
    sector_t *first_sector, sector_t *last_sector)
{
    error_t err = -EUNKNOWN;
    struct fat_node_s *node = (void*)ffile->file.node;
    struct fat_s *fat = (void*)node->node.fs;

    sector_t vsector = offset >> fat->sect_size_pow2;
    common_cluster_t vcluster = vsector >> fat->sect_per_clust_pow2;
    sector_t psector_mask = (1 << fat->sect_per_clust_pow2) - 1;
    sector_t psector_offset = vsector & psector_mask;
    common_cluster_t pcluster;

    semaphore_take(&node->lock, 1);
    pcluster = fat_file_get_cluster(ffile, vcluster);
    if ( pcluster == 0 )
        goto err;

    vfs_printk("<%s %d", __FUNCTION__, pcluster);

    *first_sector = fat->cluster0_sector + (pcluster << fat->sect_per_clust_pow2) + psector_offset;
    for (;;) {
        // Catch up as far as possible, therefore, from the zone end.
        pcluster = ffile->zone_end;
        // remember zone_end is the first cluster after contiguous zone.
        pcluster = fat->ops->entry_get(fat, pcluster-1);

        vfs_printk("-%d", pcluster);

        if ( pcluster == ffile->zone_end )
            ffile->zone_end += 1;
        else
            break;
    }
    *last_sector = fat->cluster0_sector + (ffile->zone_end << fat->sect_per_clust_pow2);

    vfs_printk(" %d>", *last_sector);

    err = 0;

  err:
    semaphore_give(&node->lock, 1);
    return err;
}

static
sector_t fat_file_get_sector(struct fat_file_s *ffile, off_t offset)
{ 
    struct fat_node_s *node = (void*)ffile->file.node;
    struct fat_s *fat = (void*)node->node.fs;

    sector_t vsector = offset >> fat->sect_size_pow2;
    common_cluster_t vcluster = vsector >> fat->sect_per_clust_pow2;
    sector_t psector_mask = (1 << fat->sect_per_clust_pow2) - 1;
    sector_t psector_offset = vsector & psector_mask;
    common_cluster_t pcluster;

    semaphore_take(&node->lock, 1);
    pcluster = fat_file_translate_cluster(ffile, vcluster);
    if ( pcluster )
        goto found;
    pcluster = fat_file_get_cluster(ffile, vcluster);
    if ( pcluster )
        goto found;
    semaphore_give(&node->lock, 1);
    return 0;

    vfs_printk("<%s: ", __FUNCTION__);
    dump_file(ffile);
    vfs_printk(" %d->%d>", pcluster, psector_offset + (pcluster << fat->sect_per_clust_pow2) + fat->cluster0_sector);

  found:
    semaphore_give(&node->lock, 1);
    return psector_offset + (pcluster << fat->sect_per_clust_pow2) + fat->cluster0_sector;
}

static
ssize_t fat_file_read_part(struct fat_file_s *ffile,
                           off_t offset,
                           void *buffer, size_t size)
{
    struct fat_node_s *node = (void*)ffile->file.node;
    struct fat_s *fat = (void*)node->node.fs;

    assert(size < (1<<fat->sect_size_pow2));

    sector_t sect = fat_file_get_sector(ffile, offset);

    if ( sect == 0 )
        return 0;

    off_t block_offset = offset & ((1 << fat->sect_size_pow2) - 1);

    vfs_printk("<%s %d bytes @ %d, block %d>", __FUNCTION__, size, block_offset, sect);
    dump_file(ffile);

    error_t err = fat_sector_lock_and_load(fat->sector, fat->dev, sect);
    if ( err )
        return -err;

    memcpy(buffer, fat->sector->data+block_offset, size);
    fat_sector_lock_release(fat->sector);

    return size;
}

static
ssize_t fat_file_read_aligned_sectors(
    struct fat_file_s *ffile,
    off_t offset,
    void *buffer, size_t size)
{
    struct fat_node_s *node = (void*)ffile->file.node;
    struct fat_s *fat = (void*)node->node.fs;
    struct device_block_s *dev = fat->dev;
    sector_t max = size >> fat->sect_size_pow2;

    sector_t begin, end;
    error_t err = fat_file_get_biggest_sector_range(ffile, offset, &begin, &end);
    if ( err )
        return err;

    max = __MIN(max, end-begin);
    uint8_t *blocks[max];
    sector_t i;
    for ( i = 0; i < max; ++i )
        blocks[i] = ((uint8_t*)buffer) + (i << fat->sect_size_pow2);

    dump_file(ffile);
    vfs_printk("<%s %d bytes @ %d, block %d, +%d>", __FUNCTION__, size, offset, begin, max);

    err = dev_block_wait_read(dev, blocks, begin, max);
    if ( err )
        return -err;

    return max << fat->sect_size_pow2;
}

#if defined(CONFIG_DRIVER_FS_FAT16)
/* If we only compile FAT32 code, we dont need this hack */
static
ssize_t fat_root_read(
    struct fat_file_s *ffile,
    off_t offset,
    void *buffer, size_t size)
{
    struct fat_node_s *node = (void*)ffile->file.node;
    struct fat_s *fat = (void*)node->node.fs;

    size_t block_size = (1 << fat->sect_size_pow2);
    size_t block_mask = (block_size - 1);
    size_t block_offset = offset & block_mask;
    sector_t sect = (offset >> fat->sect_size_pow2)
        + fat->root_dir_base;

    if ( offset >= node->file_size )
        return 0;

    if ( size + block_offset > block_size )
        size = block_size - block_offset;

    vfs_printk("<%s %d bytes @ %d, block %d>", __FUNCTION__, size, block_offset, sect);

    error_t err = fat_sector_lock_and_load(fat->sector, fat->dev, sect);
    if ( err )
        return -err;

    memcpy(buffer, fat->sector->data+block_offset, size);
    fat_sector_lock_release(fat->sector);

    return size;
}
#endif

ssize_t fat_data_read(
    struct fat_file_s *ffile,
    off_t offset,
    void *buffer, size_t size)
{
    struct fat_node_s *node = (void*)ffile->file.node;
    struct fat_s *fat = (void*)node->node.fs;

#if defined(CONFIG_DRIVER_FS_FAT16)
    if ( node->first_cluster == 0 )
        return fat_root_read(ffile, offset, buffer, size);
#endif

    uint8_t *data = buffer;
    size_t sector_size = (1 << fat->sect_size_pow2);
    size_t sector_mask = sector_size - 1;
    size_t sect_offset = offset & sector_mask;
    ssize_t left = size;

    vfs_printk("<%s %d bytes @ %d>", __FUNCTION__, size, offset);

    size_t part_size = sect_offset ? (sector_size - sect_offset) : 0;
    part_size = __MIN(size, part_size);
    ssize_t ret = 0;

    if ( part_size ) {
        ret = fat_file_read_part(ffile, offset, data, part_size);
        if (ret != part_size)
            goto err;
        data += part_size;
        left -= part_size;
        offset += part_size;
    }
    assert(((offset & sector_mask) == 0) || (left == 0));

    vfs_printk("<%s after part. left: %d bytes @ %d>", __FUNCTION__, size, offset);

    while ( left & ~sector_mask ) {
        part_size = left & ~sector_mask;
        ret = fat_file_read_aligned_sectors(
            ffile, offset, data, part_size);
        if (ret <= 0)
            goto err;
        data += ret;
        left -= ret;
        offset += ret;

        vfs_printk("<%s after aligned read. left: %d bytes @ %d>", __FUNCTION__, size, offset);
    }

    ret = 0;
    if ( left )
        ret = fat_file_read_part(ffile, offset, data, left);

  err:
    if (ret < 0)
        return ret;

    left -= ret;

    vfs_printk("<%s after all. size: %d left: %d bytes @ %d>", __FUNCTION__, size, left, offset);

    return size-left;
}

VFS_FILE_READ(fat_file_read)
{
    struct fat_file_s *ffile = (void*)file;
    struct fat_node_s *node = (void*)ffile->file.node;

    if (!(ffile->file.flags & VFS_OPEN_READ))
        return -EPERM;

    if ( file->offset >= node->file_size )
        return 0;

    size = __MIN(node->file_size - file->offset, size);

    ssize_t r = fat_data_read(ffile, file->offset, buffer, size);
    if ( r > 0 )
        file->offset += r;
    return r;
}

#if defined(CONFIG_DRIVER_FS_FAT_RW)
VFS_FILE_WRITE(fat_file_write)
{
    struct fat_file_s *ffile = (void*)file;

    if (!(ffile->file.flags & VFS_OPEN_WRITE))
        return -EPERM;

    return -ENOTSUP;
}
#endif

VFS_FILE_SEEK(fat_file_seek)
{
    struct fat_file_s *ffile = (void*)file;
    struct fat_node_s *node = (void*)ffile->file.node;

	switch (whence) {
	case VFS_SEEK_SET:
		break;
	case VFS_SEEK_CUR:
		offset += file->offset;
		break;
	case VFS_SEEK_END:
		offset += node->file_size;
		break;
	}

	if ( offset > (off_t)node->file_size )
		offset = node->file_size;
	if ( offset < 0 )
		offset = 0;

	file->offset = offset;

	return offset;
}

VFS_FILE_CLEANUP(fat_file_cleanup)
{
    struct fat_file_s *ffile = (void*)file;
    struct fat_node_s *node = (void*)file->node;

    fat_file_list_remove(&node->files, ffile);
}

static const struct vfs_file_ops_s fat_file_ops =
{
    .read = fat_file_read,
    .write = fat_file_write,
    .seek = fat_file_seek,
    .cleanup = fat_file_cleanup,
};

static const struct vfs_file_ops_s fat_dir_ops =
{
    .read = fat_dir_read,
    .cleanup = fat_file_cleanup,
};

struct fat_file_s *fat_file_create(struct fat_node_s *node,
                                   enum vfs_open_flags_e flags)
{
    struct fat_file_s *file = mem_alloc(sizeof(*file), mem_scope_sys);
    if (!file)
        return NULL;

    const struct vfs_file_ops_s *ops = NULL;

    switch (node->node.type) {
	case VFS_NODE_FILE:
        ops = &fat_file_ops;
        break;
	case VFS_NODE_DIR:
        ops = &fat_dir_ops;
		break;
    }

    if (vfs_file_init(&file->file, ops, flags, &node->node)) {
        mem_free(file);
        return NULL;
    }

    file->cluster_index = 0;
    file->zone_start = node->first_cluster;
    file->zone_end = node->first_cluster + 1;
    fat_file_list_push(&node->files, file);

    return file;
}

VFS_FS_NODE_OPEN(fat_node_open)
{
    struct fat_node_s *fnode = (void*)node;
    struct fat_file_s *ffile = fat_file_create(fnode, flags);

    if (ffile == NULL)
        return -ENOMEM;

    *file = &ffile->file;

    return 0;
}


// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
