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

#include <mutek/fileops.h>
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <vfs/file.h>
#include <vfs/node.h>

error_t vfs_file_init(struct vfs_file_s *file,
	              const struct vfs_file_ops_s *ops,
                      enum vfs_open_flags_e flags,
	              struct vfs_node_s *node)
{
  vfs_printk("<file %p init node: %p>", file, node);

  vfs_file_refinit(file);

  vfs_node_refinc(node);

  file->flags = flags;
  file->node = node;
  file->ops = ops;
  file->offset = 0;

  return 0;
}

void vfs_file_cleanup(struct vfs_file_s *file)
{
  vfs_file_refcleanup(file);
  vfs_node_refdec(file->node);

  vfs_printk("<file %p cleanup node: %p>", file, file->node);
}

void vfs_file_destroy(struct vfs_file_s *file)
{
  if (file->ops->cleanup)
    file->ops->cleanup(file);

  vfs_file_cleanup(file);

  mem_free(file);
}

ssize_t vfs_file_read(struct vfs_file_s *file,
			  void *buffer,
			  size_t size)
{
  if (!file->ops->read)
    return -ENOTSUP;
  return file->ops->read(file, buffer, size);
}

ssize_t vfs_file_write(struct vfs_file_s *file,
			   const void *buffer,
			   size_t size)
{
  if (!file->ops->write)
    return -ENOTSUP;
  return file->ops->write(file, buffer, size);
}

void vfs_file_close(struct vfs_file_s *file)
{
  vfs_file_refdec(file);
}

off_t vfs_file_seek(struct vfs_file_s *file,
			  off_t offset,
			  enum vfs_whence_e whence)
{
  if (!file->ops->seek)
    return -ENOTSUP;
  return file->ops->seek(file, offset, whence);
}

off_t vfs_file_truncate(struct vfs_file_s *file,
			  off_t new_size)
{
  if (!file->ops->truncate)
    return -ENOTSUP;
  return file->ops->truncate(file, new_size);
}
