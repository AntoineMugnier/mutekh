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

#include <hexo/types.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include "ramfs.h"
#include "ramfs-private.h"

VFS_FS_NODE_OPEN(ramfs_node_open)
{
	vfs_printk("<ramfs_node_open %p %x ", node, flags);

	if ( flags & VFS_OPEN_DIR ) {
		if ( (node->type != VFS_NODE_DIR)
			 || !(flags & VFS_OPEN_READ) )
			return -EINVAL;
	} else {
		if ( (node->type != VFS_NODE_FILE)
			 || !(flags & (VFS_OPEN_READ | VFS_OPEN_WRITE)) )
			return -EINVAL;
	}

	struct vfs_file_s *f = vfs_file_new(NULL, node);
	if ( f == NULL ) {
		vfs_printk("err>");
		return -ENOMEM;
	}

	f->close = ramfs_file_close;
	switch (node->type) {
	case VFS_NODE_FILE:
		vfs_printk("file ");
		if ( flags & VFS_OPEN_READ )
		  f->read = ramfs_file_read; /* FIXME should handle operation and return -EPERM instead ? */
		if ( flags & VFS_OPEN_WRITE )
		  f->write = ramfs_file_write;
		f->seek = ramfs_file_seek;
		f->priv = ramfs_data_refnew(node->priv);
		break;
	case VFS_NODE_DIR:
		vfs_printk("dir ");
		f->read = ramfs_dir_read;
		f->priv = (void *)(uintptr_t)0;
		break;
	}

	*file = f;
	vfs_printk("ok: %p>", f);
	return 0;
}

VFS_FILE_CLOSE(ramfs_file_close)
{
	if ( file->node->type == VFS_NODE_FILE )
		ramfs_data_refdrop(file->priv);
	vfs_file_refdrop(file);
	
	return 0;
}

VFS_FILE_READ(ramfs_file_read)
{
	struct ramfs_data_s *data = LEAF(file->node);
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
	if ( size != sizeof(struct vfs_dirent_s) )
		return -EINVAL;

	struct vfs_dirent_s *dirent = buffer;
	uintptr_t cur = (uintptr_t)file->priv;

	lock_spin(&file->node->dir.children.lock);

	if ( cur >= vfs_dir_nolock_count(&file->node->dir.children) ) {
		lock_release(&file->node->dir.children.lock);
		file->priv = (void *)(uintptr_t)0;
		return 0;
	}

	struct vfs_node_s *ent;
	size_t i;

	for ( i=0, ent = vfs_dir_nolock_head(&file->node->dir.children);
		  i<cur && ent;
		  i++, ent = vfs_dir_nolock_next(&file->node->dir.children, ent) )
		;
	file->priv = (void *)(uintptr_t)(cur+1);

	memcpy( dirent->name, ent->name, CONFIG_VFS_NAMELEN );
	dirent->type = ent->type;
	dirent->size = ent->type == VFS_NODE_FILE
		? LEAF(ent)->actual_size
		: vfs_dir_count(&ent->dir.children);;

	lock_release(&file->node->dir.children.lock);

	return sizeof(struct vfs_dirent_s);
}

VFS_FILE_WRITE(ramfs_file_write)
{
	struct ramfs_data_s *data = file->priv;
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

VFS_FILE_SEEK(ramfs_file_seek)
{
	struct ramfs_data_s *data = file->priv;

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
