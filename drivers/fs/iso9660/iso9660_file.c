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
#include <vfs/types.h>
#include <vfs/fs.h>

#include <string.h>
#include <stdlib.h>
#include <hexo/endian.h>

#include "iso9660.h"
#include "iso9660-private.h"

VFS_FILE_SEEK(iso9660_file_seek)
{
    struct iso9660_node_s *isonode = (void*)file->node;

	switch (whence) {
	case VFS_SEEK_SET:
		break;
	case VFS_SEEK_CUR:
		offset += file->offset;
		break;
	case VFS_SEEK_END:
		offset += isonode->entry.file_size;
		break;
	}

	if ( offset > isonode->entry.file_size )
		offset = isonode->entry.file_size;
	if ( offset < 0 )
		offset = 0;

	file->offset = offset;

	return offset;
}

VFS_FILE_READ(iso9660_file_read)
{
    struct iso9660_node_s *isonode = (void*)file->node;
    struct iso9660_fs_s *isofs = (void*)isonode->node.fs;
    uint8_t *buffer_ = buffer;

    /* block lba */
    dev_block_lba_t b = isonode->entry.data_blk + file->offset / ISO9660_BLOCK_SIZE;
    /* offset inside block */
    uint_fast16_t o = file->offset % ISO9660_BLOCK_SIZE;

    if (size > isonode->entry.file_size - file->offset)
        size = isonode->entry.file_size - file->offset;

    while (size) {
        error_t err;

        if ( o > 0 || size < ISO9660_BLOCK_SIZE ) {  /* use intermediate buffer for partial block read */
            uint8_t datablk[ISO9660_BLOCK_SIZE];
            uint8_t *ptr = datablk;
            /* actual small read size */
            size_t s = __MIN(size, ISO9660_BLOCK_SIZE - o);

            if (( err = dev_block_wait_read(isofs->bd, &ptr, b, 1) ))
                return err;

            memcpy(buffer_, datablk + o, s);

            buffer_ += s;
            size -= s;
            b += 1;
            o = 0;

        } else {                         /* directly read multiple blocks in buffer */
            /* read pointer table */
            uint8_t *ptr[ISO9660_BURST_BLKCOUNT];
            /* read block count */
            size_t c;

            for ( c = 0; size >= ISO9660_BLOCK_SIZE && c < ISO9660_BURST_BLKCOUNT; c++ ) {
                ptr[c] = buffer_;
                buffer_ += ISO9660_BLOCK_SIZE;
                size -= ISO9660_BLOCK_SIZE;
            }

            if (( err = dev_block_wait_read(isofs->bd, ptr, b, c) ))
                return err;

            b += c;
        }
    }

    return buffer_ - (uint8_t*)buffer;
}

VFS_FILE_READ(iso9660_dir_read)
{
    struct iso9660_node_s *isonode = (void*)file->node;
    struct iso9660_fs_s *isofs = (void*)isonode->node.fs;
    struct vfs_dirent_s *dirent = buffer;

    size_t count = ALIGN_VALUE_UP(isonode->entry.file_size, ISO9660_BLOCK_SIZE) / ISO9660_BLOCK_SIZE;
    dev_block_lba_t first = isonode->entry.data_blk;
    uint_fast16_t o = file->offset % ISO9660_BLOCK_SIZE;
    size_t b;

    for ( b = file->offset / ISO9660_BLOCK_SIZE; b < count; ) {
        error_t err;

        uint8_t dirblk[ISO9660_BLOCK_SIZE];
        uint8_t *ptr = dirblk;

        struct iso9660_dir_s *entry = (void*)(dirblk + o);

        if (( err = dev_block_wait_read(isofs->bd, &ptr, first + b, 1) ))
            return err;

        /* skip to next block on zero sized dir entry */
        if ( entry->dir_size == 0 ) {
            o = 0;
            b++;
            continue;
        }

        size_t enamelen = entry->idf_len;

        /* check entry size */
        if (sizeof(*entry) + enamelen > entry->dir_size)
            return -EIO;

        if (enamelen > 2 && entry->idf[enamelen - 2] == ';')
            enamelen -= 2;

        if (enamelen > CONFIG_VFS_NAMELEN)
            enamelen = CONFIG_VFS_NAMELEN;

        dirent->name[enamelen] = 0;
        memcpy(dirent->name, entry->idf, enamelen);

        if (entry->type & iso9660_file_isdir) {
            dirent->type = VFS_NODE_DIR;
            dirent->size = 0;
        } else {
            dirent->type = VFS_NODE_FILE;
            dirent->size = entry->file_size;
        }

        file->offset = b * ISO9660_BLOCK_SIZE + o + entry->dir_size;
        return sizeof(*dirent);
    }

    file->offset = count * ISO9660_BLOCK_SIZE;
    return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

