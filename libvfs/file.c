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

#include <mutek/fileops.h>
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

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

static VFS_FILE_TRUNCATE(default_vfs_file_truncate)
{
	return -ENOTSUP;
}

static VFS_FILE_CLOSE(default_vfs_file_close)
{
	vfs_file_refdrop(file);
	
	return 0;
}

struct vfs_file_s * vfs_file_create(struct vfs_node_s *node,
				    vfs_fs_node_refnew_t *node_refnew,
				    vfs_fs_node_refdrop_t *node_refdrop)
{
	struct vfs_file_s *obj = mem_alloc(sizeof(*obj), mem_scope_sys);

	if (obj != NULL)
	    {
		vfs_printk("<file open %p>", node);
		obj->offset = 0;
		obj->node = node_refnew(node);
		obj->node_refdrop = node_refdrop;
		obj->close = default_vfs_file_close;
		obj->read = default_vfs_file_read;
		obj->write = default_vfs_file_write;
		obj->seek = default_vfs_file_seek;
		obj->truncate = default_vfs_file_truncate;
	    }

	return obj;
}

void vfs_file_destroy(struct vfs_file_s *obj)
{
	vfs_printk("<file close %p>", obj->node);
	mem_free(obj);

	obj->node_refdrop(obj->node);
}

const struct fileops_s vfs_file_fops = {
	.read =  (fileops_read_t *)vfs_file_read,
	.write = (fileops_write_t*)vfs_file_write,
	.lseek = (fileops_lseek_t*)vfs_file_seek,
	.close = (fileops_close_t*)vfs_file_close,
};

