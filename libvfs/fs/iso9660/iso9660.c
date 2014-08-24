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

  Copyright Alexandre Becoulet, <alexandre.becoulet@free.fr>, 2009,2014
*/

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/class/block.h>
#include <device/device.h>
#include <device/driver.h>

#include <string.h>
#include <hexo/endian.h>

#include "iso9660.h"
#include "iso9660-private.h"

#include <vfs/fs.h>
#include <vfs/name.h>

struct iso9660_node_s *
iso9660_node_create(struct iso9660_fs_s *fs, const struct iso9660_dir_s *entry,
                    const char *name, size_t namelen)
{
//    const char *name = va_arg(ap, const char *);
//    size_t namelen = va_arg(ap, size_t);

    enum vfs_node_type_e type = entry->type & iso9660_file_isdir ? VFS_NODE_DIR : VFS_NODE_FILE;

    struct iso9660_node_s *node = mem_alloc(sizeof(*node), mem_scope_sys);

    if (!node)
        return NULL;

    if (vfs_node_init(&node->node, &fs->fs, type, name, namelen)) {
        mem_free(node);
        return NULL;
    }

    /* copy directory entry without file name buffer */
    memcpy(&node->entry, entry, sizeof(*entry));

    return node;
};

static const struct vfs_fs_ops_s iso9660_ops =
{
    .node_open = iso9660_node_open,
    .lookup = iso9660_lookup,
    .stat = iso9660_stat,
    .can_unmount = iso9660_can_unmount,

    .create = NULL,
    .link = NULL,
    .move = NULL,
    .unlink = NULL,
};

error_t iso9660_open(struct device_block_s *bd, struct vfs_fs_s **fs)
{
    error_t err;
    struct iso9660_fs_s *mnt;
    const struct dev_block_params_s *bdp = DEVICE_OP(bd, getparams);
    struct iso9660_node_s *root;
    uint8_t *ptr;

    vfs_printk("iso9660: opening new iso9660 volume\n");

    if ( bdp->blk_size != ISO9660_BLOCK_SIZE ) {
        vfs_printk("iso9660: unsupported device block size: %d\n", bdp->blk_size);
        return -EINVAL;
    }

    mnt = mem_alloc(sizeof (*mnt), mem_scope_sys);
    if ( mnt == NULL )
        return -ENOMEM;
    memset(mnt, 0, sizeof(*mnt));

    if ((err = vfs_fs_init(&mnt->fs, &iso9660_ops, 1)))
        goto free_mnt;

    /* read volume descriptor */
    ptr = mnt->voldesc_;
    if (( err = dev_block_spin_read(bd, &ptr, ISO9660_PRIM_VOLDESC_BLOCK, 1) )) {
        vfs_printk("iso9660: unable to read primary volume descriptor\n");
        goto fs_cleanup;
    }

    /* check signature */
    if ((mnt->voldesc.vol_desc_type != 1) ||
        strncmp(mnt->voldesc.std_identifier, "CD001", 5) ||
        (mnt->voldesc.vol_desc_version != 1)) {
        vfs_printk("iso9660: bad primary volume descriptor signature\n");
        err = -EINVAL;
        goto fs_cleanup;
    }

    /* check device size */
    if ( bdp->blk_count < mnt->voldesc.vol_blk_count ) {
        err = -EINVAL;
        vfs_printk("iso9660: device block count smaller than fs\n");
        goto fs_cleanup;
    }

    /* root node init */
    if ( ! (mnt->voldesc.root_dir.type & iso9660_file_isdir) ) {
        err = -EINVAL;
        vfs_printk("iso9660: root entry is not a directory\n");
        goto fs_cleanup;
    }

    root = iso9660_node_create(mnt, &mnt->voldesc.root_dir, "", 0);

    if (mnt->fs.root == NULL) {
        err = -ENOMEM;
        goto fs_cleanup;
    }

    mnt->bd = bd;

    vfs_fs_root_set(&mnt->fs, &root->node);

    *fs = &mnt->fs;

    return 0;
 fs_cleanup:
    vfs_fs_cleanup(&mnt->fs);
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
    struct iso9660_fs_s *isofs = (void*)ref->fs;

    size_t count = ALIGN_VALUE_UP(isonode->entry.file_size, ISO9660_BLOCK_SIZE) / ISO9660_BLOCK_SIZE;
    dev_block_lba_t first = isonode->entry.data_blk;
    size_t b;
    error_t err;

    for ( b = 0; b < count; b++ ) {

        uint8_t dirblk[ISO9660_BLOCK_SIZE];
        uint8_t *ptr = dirblk;
        struct iso9660_dir_s *entry;

        if (( err = dev_block_wait_read(isofs->bd, &ptr, first + b, 1) ))
            return err;

        for ( entry = (void*)dirblk; (uint8_t*)entry < dirblk + ISO9660_BLOCK_SIZE; ) {

            /* skip to next block on zero sized dir entry */
            if ( entry->dir_size == 0 )
                break;

            if ( (uint8_t*)entry + entry->dir_size > dirblk + ISO9660_BLOCK_SIZE ) {
                vfs_printk("iso9660: overlapping directory entry not supported\n");
                return -ENOTSUP;
            }

            /* ignore . and .. entries */
            if ( entry->idf_len > 1 || entry->idf[0] > 1 ) {

                char entryname[255];
                size_t entrynamelen = sizeof(entryname);

                if (( err = iso9660_read_direntry(isofs->bd, entry, entryname, &entrynamelen) ))
                    return err;

                vfs_name_mangle(entryname, entrynamelen, mangled_name);

                if (vfs_name_compare(entryname, entrynamelen, name, namelen)) {
                    *node = (void*)iso9660_node_create(isofs, entry, entryname, entrynamelen);
                    return *node ? 0 : -ENOMEM;
                }
            }

            entry = (void *) ((uint8_t*)entry + entry->dir_size);
        }

    }

    return -ENOENT;
}

static const struct vfs_file_ops_s iso9660_file_ops =
{
    .read = iso9660_file_read,
    .seek = iso9660_file_seek,
};

static const struct vfs_file_ops_s iso9660_dir_ops =
{
    .read = iso9660_dir_read,
};

VFS_FS_NODE_OPEN(iso9660_node_open)
{
    assert(!(flags & VFS_OPEN_WRITE));

    struct vfs_file_s *f = mem_alloc(sizeof (*f), mem_scope_sys);
    if (f == NULL)
        return -ENOMEM;

    const struct vfs_file_ops_s *ops = NULL;

    switch (node->type) {
	case VFS_NODE_FILE:
        ops = &iso9660_file_ops;
        break;
	case VFS_NODE_DIR:
        ops = &iso9660_dir_ops;
		break;
    }

    error_t err = vfs_file_init(f, ops, flags, node);

    if (err) {
        mem_free(f);
        return err;
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

