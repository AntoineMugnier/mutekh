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
    Copyright Sylvain Leroy <sylvain.leroy@unmondelibre.fr>
*/

#ifndef __DEVFS_H__
#define __DEVFS_H__

#include <hexo/types.h>
#include <vfs/vfs.h>
#include "devfs-private.h"


/**
 ** add an inode in /dev
 */
error_t	devfs_register(char*			name,
		       enum devfs_type_e	type);
/**
 ** remove an inode in /dev
 */
error_t	devfs_unregister(char*	name);

/**
 ** initialisation of the devfs
 */
error_t	devfs_init(struct vfs_node_s	*root);


VFS_CREATE_CONTEXT(devfs_create_context);
VFS_DESTROY_CONTEXT(devfs_destroy_context);
VFS_READ_ROOT(devfs_read_root);
VFS_WRITE_ROOT(devfs_write_root);

VFS_INIT_NODE(devfs_init_node);
VFS_CREATE_NODE(devfs_create_node);
VFS_LOOKUP_NODE(devfs_lookup_node);
VFS_WRITE_NODE(devfs_write_node);
VFS_RELEASE_NODE(devfs_release_node);
VFS_UNLINK_NODE(devfs_unlink_node);

VFS_OPEN_FILE(devfs_open);
VFS_READ_FILE(devfs_read);
VFS_WRITE_FILE(devfs_write);
VFS_LSEEK_FILE(devfs_lseek);
VFS_RELEASE_FILE(devfs_release);
VFS_READ_DIR(devfs_readdir);

static const struct vfs_context_op_s devfs_ctx_op =
  {
    .create  = devfs_create_context,
    .destroy = devfs_destroy_context,
    .read_root = devfs_read_root,
    .write_root = devfs_write_root
  };

static const struct vfs_node_op_s devfs_n_op =
  {
    .init = devfs_init_node,
    .create = devfs_create_node,
    .lookup = devfs_lookup_node,
    .write = devfs_write_node,
    .release = devfs_release_node,
    .unlink = devfs_unlink_node
  };
static const struct vfs_file_op_s devfs_f_op =
  {
    .open = devfs_open,
    .read = devfs_read,
    .write = devfs_write,
    .lseek = devfs_lseek,
    .readdir = devfs_readdir,
    .release = devfs_release
  };

#endif /* __DEVFS_H__ */
