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
  struct nfs_context_s *nfs_pv = NULL;

#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_create_context : Allocating private stuff\n");
#endif

  // Allocating memory for parent context
  if((nfs_pv = mem_alloc(sizeof(struct nfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  if((nfs_pv->server = mem_alloc(sizeof(struct nfs_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  // Link private struct to context private field
  context->ctx_pv = nfs_pv;

  return NFS_OK;
}


/**
 ** param	struct vfs_context_s *context
 ** return	error_t
 */
VFS_DESTROY_CONTEXT(nfs_destroy_context)
{
  struct nfs_context_s *nfs_pv = context->ctx_pv;

#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_destroy_context : Freeing private stuff\n");
#endif

  mem_free(nfs_pv->server);
  mem_free(context->ctx_pv);

  return NFS_OK;
}


VFS_READ_ROOT(nfs_read_root)
{
#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_read_root: This function should not be called\n");
#endif

  return 0;
}


VFS_WRITE_ROOT(nfs_write_root)
{
#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_write_root: This function should not be called\n");
#endif

  return 0;
}
