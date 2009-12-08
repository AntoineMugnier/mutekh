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

#ifndef __VFAT_H__
#define __VFAT_H__

#include <vfs/vfs.h>


VFS_CREATE_CONTEXT(vfat_create_context);
VFS_DESTROY_CONTEXT(vfat_destroy_context);
VFS_READ_ROOT(vfat_read_root);
VFS_WRITE_ROOT(vfat_write_root);


VFS_INIT_NODE(vfat_init_node);
VFS_CREATE_NODE(vfat_create_node);
VFS_LOOKUP_NODE(vfat_lookup_node);
VFS_WRITE_NODE(vfat_write_node);
VFS_RELEASE_NODE(vfat_release_node);
VFS_UNLINK_NODE(vfat_unlink_node);


VFS_OPEN_FILE(vfat_open);
#if defined(CONFIG_DRIVER_FS_VFAT_READBYPASSBC)
VFS_READ_FILE(vfat_read_bypass_bc);
#else
VFS_READ_FILE(vfat_read);
#endif
VFS_WRITE_FILE(vfat_write);
VFS_LSEEK_FILE(vfat_lseek);
VFS_RELEASE_FILE(vfat_release);
VFS_READ_DIR(vfat_readdir);


static const struct vfs_context_op_s vfat_ctx_op = 
  {
    .create  = vfat_create_context,
    .destroy = vfat_destroy_context,
    .read_root = vfat_read_root,
    .write_root = vfat_write_root
  };

static const struct vfs_node_op_s vfat_n_op = 
  {
    .init = vfat_init_node,
    .create = vfat_create_node,
    .lookup = vfat_lookup_node,
    .write = vfat_write_node,
    .release = vfat_release_node,
    .unlink = vfat_unlink_node
  };

static const struct vfs_file_op_s vfat_f_op = 
  {
    .open = vfat_open,
#if defined(CONFIG_DRIVER_FS_VFAT_READBYPASSBC)
    .read = vfat_read_bypass_bc,
#else
    .read = vfat_read,
#endif
    .write = vfat_write,
    .lseek = vfat_lseek,
    .readdir = vfat_readdir,
    .release = vfat_release
  };

#endif
