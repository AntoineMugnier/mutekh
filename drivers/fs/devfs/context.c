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

#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include <hexo/endian.h>
#include <vfs/vfs.h>
#include <vfs/buffer_cache.h>
#include "devfs.h"



VFS_CREATE_CONTEXT(devfs_create_context)
{
  context->ctx_type = VFS_DEVFS_TYPE;
  context->ctx_op = (struct vfs_context_op_s *) &devfs_ctx_op;
  context->ctx_node_op = (struct vfs_node_op_s *) &devfs_n_op;
  context->ctx_file_op = (struct vfs_file_op_s *) &devfs_f_op;
  context->ctx_pv = NULL;
  return 0;
}


VFS_DESTROY_CONTEXT(devfs_destroy_context)
{
  return 0;
}


VFS_READ_ROOT(devfs_read_root)
{
  return 0;
}


VFS_WRITE_ROOT(devfs_write_root)
{
  return 0;
}
