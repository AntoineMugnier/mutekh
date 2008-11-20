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


/**
 ** param	struct vfs_context_s *context
 ** return	error_t
 */

VFS_CREATE_CONTEXT(devfs_create_context)
{
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
