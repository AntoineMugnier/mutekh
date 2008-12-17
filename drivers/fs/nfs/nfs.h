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

#ifndef __NFS_H__
#define __NFS_H__

#include <vfs/vfs.h>

/*
** brief mount a NFS in "mount_point" linked to "server_ip"
**
** param location to mount the NFS in the VFS
** param to which server it should be linked
**
** return error code (0 = OK, other = KO)
*/
error_t	nfs_mount(const char		*mount_point,
		  uint_fast32_t	server_ip);

/*
** brief unmount the NFS mounted in "mount_point"
**
** param location of the mounted NFS that must be unmounted
**
** return error code (0 = OK, other = KO)
*/
error_t	nfs_umount(const char		*mount_point);


VFS_CREATE_CONTEXT(nfs_create_context);
VFS_DESTROY_CONTEXT(nfs_destroy_context);
VFS_READ_ROOT(nfs_read_root);
VFS_WRITE_ROOT(nfs_write_root);

VFS_INIT_NODE(nfs_init_node);
VFS_CREATE_NODE(nfs_create_node);
VFS_LOOKUP_NODE(nfs_lookup_node);
VFS_WRITE_NODE(nfs_write_node);
VFS_RELEASE_NODE(nfs_release_node);
VFS_UNLINK_NODE(nfs_unlink_node);

VFS_OPEN_FILE(nfs_open);
VFS_READ_FILE(nfs_read);
VFS_WRITE_FILE(nfs_write);
VFS_LSEEK_FILE(nfs_lseek);
VFS_RELEASE_FILE(nfs_release);
VFS_READ_DIR(nfs_readdir);

static const struct vfs_context_op_s nfs_ctx_op =
  {
    .create  = nfs_create_context,
    .destroy = nfs_destroy_context,
    .read_root = nfs_read_root,
    .write_root = nfs_write_root
  };

static const struct vfs_node_op_s nfs_n_op =
  {
    .init = nfs_init_node,
    .create = nfs_create_node,
    .lookup = nfs_lookup_node,
    .write = nfs_write_node,
    .release = nfs_release_node,
    .unlink = nfs_unlink_node
  };
static const struct vfs_file_op_s nfs_f_op =
  {
    .open = nfs_open,
    .read = nfs_read,
    .write = nfs_write,
    .lseek = nfs_lseek,
    .readdir = nfs_readdir,
    .release = nfs_release
  };

#endif /* __NFS_H__ */
