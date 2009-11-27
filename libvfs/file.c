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

VFS_FILE_READ(vfs_file_read_na)
{
	return -ENOTSUP;
}

VFS_FILE_WRITE(vfs_file_write_na)
{
	return -ENOTSUP;
}

VFS_FILE_SEEK(vfs_file_seek_na)
{
	return -ENOTSUP;
}

OBJECT_CONSTRUCTOR(vfs_file)
{
	struct vfs_node_s *node = vfs_node_refnew(va_arg(ap, struct vfs_node_s*));
	vfs_printk("<file create %s>", node->name);
	obj->offset = 0;
	obj->node = node;

	atomic_inc(&node->fs->ref);

	return 0;
}

OBJECT_DESTRUCTOR(vfs_file)
{
	vfs_printk("<file delete %s>", obj->node->name);

	atomic_dec(&obj->node->fs->ref);

	vfs_node_refdrop(obj->node);
}

