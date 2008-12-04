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

#ifndef __PIPE_H__
#define __PIPE_H__

#include <hexo/types.h>
#include <mutek/scheduler.h>
#include <hexo/lock.h>
#include <vfs/vfs.h>
#include <vfs/buffer_cache.h>

#define PIPE_DEBUG 0

struct pipe_node_s
{
  size_t size;
  size_t status;
  size_t pRead;
  size_t pWrite;
  uint_fast16_t flags;
  sched_queue_root_t wr_wait;
  sched_queue_root_t rd_wait;
  struct rwlock_s lock;
  struct bc_buffer_s *buffer;
};

VFS_CREATE_CONTEXT(pipe_create_context);
VFS_DESTROY_CONTEXT(pipe_destroy_context);
VFS_READ_ROOT(pipe_read_root);
VFS_WRITE_ROOT(pipe_write_root);

VFS_INIT_NODE(pipe_init_node);
VFS_CREATE_NODE(pipe_create_node);
VFS_LOOKUP_NODE(pipe_lookup_node);
VFS_WRITE_NODE(pipe_write_node);
VFS_RELEASE_NODE(pipe_release_node);
VFS_UNLINK_NODE(pipe_unlink_node);

VFS_OPEN_FILE(pipe_open);
VFS_READ_FILE(pipe_read);
VFS_WRITE_FILE(pipe_write);
VFS_LSEEK_FILE(pipe_lseek);
VFS_RELEASE_FILE(pipe_release);
VFS_READ_DIR(pipe_readdir);

static const struct vfs_context_op_s pipe_ctx_op = 
  {
    .create  = pipe_create_context,
    .destroy = pipe_destroy_context,
    .read_root = pipe_read_root,
    .write_root = pipe_write_root
  };

static const struct vfs_node_op_s pipe_n_op = 
  {
    .init = pipe_init_node,
    .create = pipe_create_node,
    .lookup = pipe_lookup_node,
    .write = pipe_write_node,
    .release = pipe_release_node,
    .unlink = pipe_unlink_node
  };

static const struct vfs_file_op_s pipe_f_op = 
  {
    .open = pipe_open,
    .read = pipe_read,
    .write = pipe_write,
    .lseek = pipe_lseek,
    .readdir = pipe_readdir,
    .release = pipe_release
  };

#endif
