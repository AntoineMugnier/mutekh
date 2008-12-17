/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    UPMC / LIP6 / SOC (c) 2008
    Copyright Ghassan Almaless <ghassan.almaless@gmail.com>
*/

#ifndef __VFS_PRIVATE_H_
#define __VFS_PRIVATE_H_


///

uint_fast8_t vfs_dir_count(char *path);

void vfs_split_path(char *path, char **dirs);

///

#define VFS_NODE_FREELIST_INIT(n) error_t (n) (struct vfs_context_s *ctx,\
					  uint_fast8_t length)

#define VFS_NODE_FREELIST_ADD(n) void (n) (struct vfs_node_s *node,\
				      bool_t hasError)

#define VFS_NODE_FREELIST_GET(n) struct vfs_node_s* (n) (struct vfs_context_s* parent_ctx)

#define VFS_NODE_FREELIST_UNLINK(n) void (n) (struct vfs_node_s *node)

#define VFS_FILE_FREELIST_INIT(n) error_t (n) (uint_fast8_t length)
#define VFS_FILE_FREELIST_ADD(n)  void (n) (struct vfs_file_s *file)
#define VFS_FILE_FREELIST_GET(n) struct vfs_file_s * (n) (struct vfs_node_s *node)


#define VFS_NODE_INIT(n) error_t (n) (struct vfs_context_s *ctx,\
				      struct vfs_node_s *node)

#define VFS_NODE_LOOKUP(n) struct vfs_node_s* (n) (struct vfs_node_s *node,\
						   char *name)

#define VFS_NODE_UP(n) void (n) (struct vfs_node_s *node)

#define VFS_NODE_DOWN(n) void (n) (struct vfs_node_s *node)

#define VFS_NODE_LOAD(n) error_t (n) (struct vfs_node_s *root,	\
				      char **path,		\
				      uint_fast32_t flags,	\
				      bool_t isAbsolutePath,	\
				      struct vfs_node_s **node)

#define VFS_NODE_CREATE(n) error_t (n) (struct vfs_node_s *parent,	\
					uint_fast32_t flags,		\
					bool_t isLast,			\
					struct vfs_node_s *node)
					

VFS_NODE_FREELIST_INIT(vfs_node_freelist_init);
VFS_NODE_FREELIST_ADD(vfs_node_freelist_add);
VFS_NODE_FREELIST_GET(vfs_node_freelist_get);
VFS_NODE_FREELIST_UNLINK(vfs_node_freelist_unlink);

VFS_FILE_FREELIST_INIT(vfs_file_freelist_init);
VFS_FILE_FREELIST_ADD(vfs_file_freelist_add);
VFS_FILE_FREELIST_GET(vfs_file_freelist_get);

VFS_NODE_INIT(vfs_node_init);
VFS_NODE_LOOKUP(vfs_node_lookup);
VFS_NODE_UP(vfs_node_up);
VFS_NODE_DOWN(vfs_node_down);
VFS_NODE_LOAD(vfs_node_load);
VFS_NODE_CREATE(vfs_node_create);

#endif
