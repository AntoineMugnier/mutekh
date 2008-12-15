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

#include <hexo/alloc.h>
#include <vfs/vfs.h>
#include "nfs.h"
#include "nfs-private.h"


/**
 ** param	struct vfs_context_s *context
 ** return	error_t
 */

VFS_CREATE_CONTEXT(nfs_create_context)
{
  // Allocating memory for parent context
  if((context->ctx_pv = mem_alloc(sizeof(struct nfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  return NFS_OK;
}


VFS_DESTROY_CONTEXT(nfs_destroy_context)
{
  mem_free(context);

  return NFS_OK;
}


VFS_READ_ROOT(nfs_read_root)
{
  printf("nfs_read_root: This function should not be called\n");
  return 0;
}


VFS_WRITE_ROOT(nfs_write_root)
{
  printf("nfs_write_root: This function should not be called\n");
  return 0;
}
