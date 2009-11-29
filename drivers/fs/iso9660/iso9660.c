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

  Copyright Alexandre Becoulet, <alexandre.becoulet@free.fr>, 2009
*/

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/block.h>
#include <device/device.h>
#include <device/driver.h>

#include <string.h>

#include "iso9660.h"
#include "iso9660-private.h"

static void iso9660_node_delete(struct vfs_node_s *node)
{
    mem_free(node);
}

static struct iso9660_node_s *
iso9660_node_new(struct vfs_fs_s *fs, const struct iso9660_dir_s *entry)
{
    struct iso9660_node_s *isonode = mem_alloc(sizeof (*isonode), mem_scope_sys);
    enum vfs_node_type_e type = isonode->entry.type & iso9660_file_isdir ? VFS_NODE_DIR : VFS_NODE_FILE;

    if (vfs_node_new(&isonode->node, fs, type, entry->idf,
                     entry->idf_len, NULL, iso9660_node_delete))
        {
            mem_free(isonode);
            return NULL;
        }

    /* copy directory entry without file name buffer */
    memcpy(&isonode->entry, entry, sizeof(*entry));

    return isonode;
};

error_t iso9660_open(struct vfs_fs_s **fs, struct device_s *bd)
{
    error_t err;
    struct iso9660_fs_s *mnt;
    const struct dev_block_params_s *bdp = dev_block_getparams(bd);
    uint8_t *ptr;

    if ( bdp->blk_size != ISO9660_BLOCK_SIZE )
		return -EINVAL;

    mnt = mem_alloc(sizeof (*mnt), mem_scope_sys);
    if ( mnt == NULL )
        return -ENOMEM;
    memset(mnt, 0, sizeof(*mnt));

    /* read volume descriptor */
    ptr = mnt->voldesc_;
    if (( err = dev_block_spin_read(bd, &ptr, ISO9660_PRIM_VOLDESC_BLOCK, 1) ))
        goto free_mnt;

    /* check signature */
    if ((mnt->voldesc.vol_desc_type != 1) ||
        strncmp(mnt->voldesc.std_identifier, "CD001", 5) ||
        (mnt->voldesc.vol_desc_version != 1)) {
        err = -EINVAL;
        goto free_mnt;
    }

    /* check device size */
    if ( bdp->blk_count < mnt->voldesc.vol_blk_count ) {
        err = -EINVAL;
        goto free_mnt;
    }

    /* fs struct */
    atomic_set(&mnt->fs.ref, 0);
    mnt->fs.node_open = iso9660_node_open;
    mnt->fs.lookup = iso9660_lookup;
    mnt->fs.stat = iso9660_stat;
    mnt->fs.can_unmount = iso9660_can_unmount;
    mnt->fs.flag_ro = 1;
    mnt->fs.old_node = NULL;

    /* root node init */
    if ( ! (mnt->voldesc.root_dir.type & iso9660_file_isdir) ) {
        err = -EINVAL;
        goto free_mnt;
    }

    mnt->fs.root = (struct vfs_node_s *)iso9660_node_new((struct vfs_fs_s*)mnt, &mnt->voldesc.root_dir);

    if (mnt->fs.root == NULL)
        goto free_mnt;

    mnt->bd = device_obj_refnew(bd);

	// TODO register destructor

	*fs = (struct vfs_fs_s*)mnt;

	return 0;

 free_mnt:
	mem_free(mnt);
	return err;
}

VFS_FS_CAN_UNMOUNT(iso9660_can_unmount)
{
	return 0;
}

VFS_FS_LOOKUP(iso9660_lookup)
{
    struct iso9660_node_s *isonode = (void*)ref;
    struct iso9660_fs_s *isofs = (void*)isonode->node.fs;

    size_t count = ALIGN_VALUE_UP(isonode->entry.file_size, ISO9660_BLOCK_SIZE) / ISO9660_BLOCK_SIZE;
    dev_block_lba_t first = isonode->entry.data_blk;
    size_t b;
    error_t err;

    for ( b = 0; b < + count; b++ ) {

        uint8_t dirblk[ISO9660_BLOCK_SIZE];
        uint8_t *ptr = dirblk;
        struct iso9660_dir_s *entry;

        if (( err = dev_block_wait_read(isofs->bd, &ptr, first + b, 1) ))
            return err;

        for ( entry = (void*)dirblk; (uint8_t*)entry < dirblk + 1; ) {

            /* skip to next block on zero sized dir entry */
            if ( entry->dir_size == 0 )
                break;

            size_t enamelen = entry->idf_len;

            /* check entry size */
            if (sizeof(*entry) + enamelen > entry->dir_size)
                return -EIO;

            if (enamelen > 2 && entry->idf[enamelen - 2] == ';')
                enamelen -= 2;

            if (enamelen == namelen && !memcmp(entry->idf, name, namelen)) {
                *node = (void*)iso9660_node_new(ref->fs, entry);
                return 0;
            }

            entry = (void *) ((uint8_t*)entry + entry->dir_size);
        }

    }

    return -ENOENT;
}

VFS_FS_NODE_OPEN(iso9660_node_open)
{
    struct vfs_file_s *f = vfs_file_new(NULL, node);

    assert(! (flags & VFS_OPEN_WRITE) );

    if (!f)
        return -ENOMEM;

    if ( flags & VFS_OPEN_DIR ) {
        f->read = iso9660_dir_read;
    } else {
        f->read = iso9660_file_read;
        f->seek = iso9660_file_seek;
    }

    *file = f;
    return 0;
}


VFS_FS_STAT(iso9660_stat)
{
    struct iso9660_node_s *isonode = (void*)node;

    stat->nlink = 1;
    stat->size = isonode->entry.file_size;
    stat->type = isonode->entry.type & iso9660_file_isdir ? VFS_NODE_DIR : VFS_NODE_FILE;

    return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

