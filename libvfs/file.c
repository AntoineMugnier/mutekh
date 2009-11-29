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

#include <vfs/vfs.h>

static VFS_FILE_READ(default_vfs_file_read)
{
	return -ENOTSUP;
}

static VFS_FILE_WRITE(default_vfs_file_write)
{
	return -ENOTSUP;
}

static VFS_FILE_SEEK(default_vfs_file_seek)
{
	return -ENOTSUP;
}

static VFS_FILE_CLOSE(default_vfs_file_close)
{
	vfs_file_refdrop(file);
	
	return 0;
}

OBJECT_CONSTRUCTOR(vfs_file)
{
	struct vfs_node_s *node = vfs_node_refnew(va_arg(ap, struct vfs_node_s*));
	vfs_printk("<file open %s>", node->name);
	obj->offset = 0;
	obj->node = node;
	obj->close = default_vfs_file_close;
	obj->read = default_vfs_file_read;
	obj->write = default_vfs_file_write;
	obj->seek = default_vfs_file_seek;

	atomic_inc(&node->fs->ref);

	return 0;
}

OBJECT_DESTRUCTOR(vfs_file)
{
	vfs_printk("<file close %s>", obj->node->name);

	atomic_dec(&obj->node->fs->ref);

	vfs_node_refdrop(obj->node);
}

