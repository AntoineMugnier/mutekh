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

// Public define
// Values for  devfs_node_s->type;
#define DEVFS_DIR		0x00
#define DEVFS_CHAR		0x01
#define DEVFS_BLOCK		0x02

/**
 ** \brief	initialisation and mount of the devfs
 ** \param	mount_point for the DevFS
 ** \return	error value
 */
error_t	devfs_init(const char		*mount_point);

/**
 ** \brief	add an inode to /dev
 ** \param	name of the node in the FS
 ** \param	structure of the device
 ** \param	type if : DEVFS_CHAR or DEVFS_BLOCK
 ** \return	structure of the node registered
 */
struct devfs_node_s	*devfs_register(const char		*name,
					struct device_s		*device,
					uint_fast8_t		type);

/**
 ** \brief	remove an inode from /dev
 ** \param	name of the node to unregister
 ** \return	error value
 */
error_t	devfs_unregister(const char		*name);

/**
 ** \brief	umount the devfs
 ** \param	path to the already mounted DevFS
 ** \return	error value
 */
error_t	devfs_destroy(const char		*mount_point);

////////////////////////////////////////////////////////////////

/**
 ** \brief	Context initialization
 ** \param	struct vfs_context_s *context
 ** \return	error_t
 */
VFS_CREATE_CONTEXT(devfs_create_context);

/**
 ** \brief	Context destruction
 ** \param	struct vfs_context_s *context
 ** \return	error_t
 */
VFS_DESTROY_CONTEXT(devfs_destroy_context);

/**
 ** \brief	Not used
 ** \param	struct vfs_context_s	*context
 ** \param	struct vfs_node_s	*root
 ** \return	error_t
 */
VFS_READ_ROOT(devfs_read_root);

/**
 ** \brief	Not used
 ** \param	struct vfs_context_s	*context
 ** \param	struct vfs_node_s	*root
 ** \return	error_t
 */
VFS_WRITE_ROOT(devfs_write_root);

////////////////////////////////////////////////////////////////

/**
 ** \brief	Initialize a node (not used)
 ** \param	struct vfs_node_s *node
 ** \return	error_t
 */
VFS_INIT_NODE(devfs_init_node);

/**
 ** \brief	Create a node (not used)
 ** \param	struct vfs_node_s *parent
 ** \param	struct vfs_node_s *node
 ** \return	error_t
 */
VFS_CREATE_NODE(devfs_create_node);

/**
 ** \brief	lookup for a node
 ** \param	struct vfs_node_s *parent
 ** \param	struct vfs_node_s *node
 ** \return	error_t
 */
VFS_LOOKUP_NODE(devfs_lookup_node);

/**
 ** \brief	Not used (not used)
 ** \param	struct vfs_node_s *node
 ** \return	error_t
 */
VFS_WRITE_NODE(devfs_write_node);

/**
 ** \brief	Release a node (not used)
 ** \param	struct vfs_node_s *node
 ** \return	error_t
 */
VFS_RELEASE_NODE(devfs_release_node);

/**
 ** \brief	unlink a node (not used)
 ** \param	struct vfs_node_s *node
 ** \return	error_t
 */
VFS_UNLINK_NODE(devfs_unlink_node);

////////////////////////////////////////////////////////////////

/**
 ** \brief	open a device in a file structure
 ** \param	struct vfs_node_s *node
 ** \param	struct vfs_file_s *file
 ** \return	error_t
 */
VFS_OPEN_FILE(devfs_open);

/**
 ** \brief	read on a device (char or block)
 ** \param	struct vfs_file_s	*file
 ** \param	uint8_t			*buffer
 ** \param	size_t			size
 ** \return	error_t
 */
VFS_READ_FILE(devfs_read);

/**
 ** \brief	write on a device (char or block)
 ** \param	struct vfs_file_s	*file
 ** \param	uint8_t			*buffer
 ** \param	size_t			size
 ** \return	error_t
 */
VFS_WRITE_FILE(devfs_write);

/**
 ** \brief	not used
 ** \param	struct vfs_file_s	*file
 ** \return	error_t
 */
VFS_LSEEK_FILE(devfs_lseek);

/**
 ** \brief	not used
 ** \param	struct vfs_file_s	*file
 ** \return	error_t
 */
VFS_RELEASE_FILE(devfs_release);

/**
 ** \brief	not read a directory from the DevFS
 ** \param	struct vfs_file_s	*file
 ** \param	struct vfs_dirent_s	*dirent
 ** \return	error_t
 */
VFS_READ_DIR(devfs_readdir);

////////////////////////////////////////////////////////////////

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
