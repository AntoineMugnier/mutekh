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

#include <hexo/types.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <vfs/node.h>
#include <vfs/file.h>

#include "ramfs_file.h"
#include "ramfs-private.h"
#include "ramfs_data.h"

VFS_FILE_READ(ramfs_file_read)
{
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)file->node;
	struct ramfs_data_s *data = rnode->data;
	ssize_t left = data->actual_size - file->offset;
	if ( size < left )
		left = size;
	if ( left < 0 )
		return -EINVAL;

	memcpy(buffer, (void*)(((uintptr_t)data->data)+file->offset), left);

	file->offset += left;

	return left;
}

VFS_FILE_READ(ramfs_dir_read)
{
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)file->node;

	if ( size != sizeof(struct vfs_dirent_s) )
		return -EINVAL;

	uintptr_t cur = (uintptr_t)rnode->data;

    bool_t gotit = ramfs_dir_get_nth(rnode, buffer, cur);

    if ( gotit )
        rnode->data = (void *)(uintptr_t)(cur+1);
    else
        rnode->data = (void *)(uintptr_t)0;

	return gotit ? sizeof(struct vfs_dirent_s) : 0;
}

VFS_FILE_WRITE(ramfs_file_write)
{
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)file->node;
	struct ramfs_data_s *data = rnode->data;

    if (file->flags & VFS_OPEN_APPEND)
        file->offset = data->actual_size;

	ssize_t offset_after = file->offset + size;

	if ( offset_after > (off_t)data->allocated_size ) {
		size_t new_size = offset_after;
		new_size |= 0xfff;
		new_size += 1;

		error_t err = ramfs_data_realloc(data, new_size);
		if ( err )
			return err;
	}

	memcpy((void*)(((uintptr_t)data->data)+file->offset), buffer, size);

	file->offset += size;
	if ( file->offset > (off_t)data->actual_size )
		data->actual_size = file->offset;

	return size;
}

VFS_FILE_TRUNCATE(ramfs_file_truncate)
{
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)file->node;
    struct ramfs_data_s *data = rnode->data;

    if ( new_size > (off_t)data->allocated_size ) {
        new_size |= 0xfff;
        new_size += 1;

        error_t err = ramfs_data_realloc(data, new_size);
        if ( err )
            return err;

        memset(data->data+data->actual_size, 0, new_size - data->actual_size);
    }

    data->actual_size = new_size;

    return 0;
}

VFS_FILE_SEEK(ramfs_file_seek)
{
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)file->node;
	struct ramfs_data_s *data = rnode->data;

	switch (whence) {
	case VFS_SEEK_SET:
		break;
	case VFS_SEEK_CUR:
		offset += file->offset;
		break;
	case VFS_SEEK_END:
		offset += data->actual_size;
		break;
	}

	if ( offset > (off_t)data->actual_size )
		offset = data->actual_size;
	if ( offset < 0 )
		offset = 0;

	file->offset = offset;

	return offset;
}

static const struct vfs_file_ops_s ramfs_file_ops =
{
    .read = ramfs_file_read,
    .write = ramfs_file_write,
    .truncate = ramfs_file_truncate,
    .seek = ramfs_file_seek,
};

static const struct vfs_file_ops_s ramfs_dir_ops =
{
    .read = ramfs_dir_read,
};

VFS_FS_NODE_OPEN(ramfs_node_open)
{
    __unused__ struct ramfs_node_s *rnode = (struct ramfs_node_s *)node;

	vfs_printk("<ramfs_node_open %p %x ", rnode, flags);

	struct vfs_file_s *f = mem_alloc(sizeof(*f), mem_scope_sys);
	if ( f == NULL ) {
		vfs_printk("err>");
		return -ENOMEM;
	}

    const struct vfs_file_ops_s *ops = NULL;

    switch (node->type) {
	case VFS_NODE_FILE:
        ops = &ramfs_file_ops;
        if ( flags & VFS_OPEN_TRUNCATE )
            ramfs_file_truncate(f, 0);
        break;
	case VFS_NODE_DIR:
        ops = &ramfs_dir_ops;
		break;
    }

    error_t err = vfs_file_init(f, ops, flags, node);
    if (err) {
		vfs_printk("err %d>", err);
        mem_free(f);
        return err;
    }

	*file = f;
	vfs_printk("ok: %p>", f);
	return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
