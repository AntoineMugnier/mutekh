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
#include <vfs/vfs-private.h>
#include <netinet/nfs.h>
//#include <netinet/packet.h>
#include <assert.h>

#include "nfs.h"
#include "nfs-private.h"


error_t	nfs_mount(const char		*mount_point,
		  uint_fast32_t		server_ip)
{
  struct vfs_context_s		*nfs_ctx = NULL;
  struct vfs_node_s		*node = NULL;
  error_t			err = 0;
  uint_fast32_t			flags = 0;
  struct nfs_context_s		*nfs_pv = NULL;
  char				*dirs_ptr[vfs_dir_count(mount_point) + 1];
  bool_t			isAbsolutePath = 1;

  assert(mount_point != NULL);

#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_mount : Mounting NFS in %s\n", mount_point);
#endif

  // Some check
  if (mount_point[0] != '/')
    {
      printf("nfs_mount : mount_point must be absolute, abording\n");
      return NFS_ERR;
    }

  // translate from char* to char**
  vfs_split_path(mount_point, dirs_ptr);

  // Setting up flags
  VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_EXCL | VFS_DIR);

  // Get the node if existing
  if((err = vfs_node_load(vfs_get_root(), dirs_ptr, flags, isAbsolutePath, &node)))
    {
      printf("nfs_init: %s doesn't seem to exist in filesystem, abording\n", mount_point);
      return err;
    }

  // Allocating memory for parent context
  if((nfs_ctx = mem_alloc(sizeof(struct vfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  // Setting up vfs_context_s
  nfs_ctx->ctx_type = VFS_NFS_TYPE;
  nfs_ctx->ctx_dev = NULL;
  nfs_ctx->ctx_op = (struct vfs_context_op_s *) &nfs_ctx_op;
  nfs_ctx->ctx_node_op = (struct vfs_node_op_s *) &nfs_n_op;
  nfs_ctx->ctx_file_op = (struct vfs_file_op_s *) &nfs_f_op;
  nfs_ctx->ctx_pv = NULL;

  // Create intern stuff
  if (nfs_ctx->ctx_op->create(nfs_ctx))
    return NFS_ERR;

  // Get private context into a usable variable (not void*)
  nfs_pv = nfs_ctx->ctx_pv;

  //////////////

  /* Init NFS */
  memset(&nfs_pv->server, 0, sizeof (nfs_pv->server));
  IPV4_ADDR_SET(nfs_pv->server->address, server_ip);
  nfs_pv->server->uid = 500;
  nfs_pv->server->gid = 500;
  net_nfs_init(nfs_pv->server);

  if (net_nfs_mount(nfs_pv->server, mount_point, nfs_pv->root))
    {
      printf("nfs_mount : cannot mount nfs\n");
      return NFS_ERR;
    }

  //////////////

  // Change context type of mount_point to NFS
  node->n_ctx = nfs_ctx;

#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_mount : NFS Initialized\n");
#endif

  return NFS_OK;
}

////////////////////////////////////////////////////

error_t	nfs_umount(const char		*mount_point)
{
  struct vfs_node_s		*node = NULL;
  struct nfs_context_s		*nfs_pv = NULL;
  char				*dirs_ptr[vfs_dir_count(mount_point) + 1];
  bool_t			isAbsolutePath = 1;
  error_t			err = 0;
  uint_fast32_t			flags = 0;

#ifdef CONFIG_DRIVER_FS_NFS_DEBUG
  printf("nfs_umount : umounting NFS\n");
#endif

  // Some check
  if (mount_point[0] != '/')
    {
      printf("nfs_umount : mount_point must be absolute, abording\n");
      return NFS_ERR;
    }

  // translate from char* to char**
  vfs_split_path(mount_point, dirs_ptr);

  // Setting up flags
  VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_EXCL | VFS_DIR);

  // Get the node if existing
  if((err = vfs_node_load(vfs_get_root(), dirs_ptr, flags, isAbsolutePath, &node)))
    return err;

  // downcount node refcount
  // Twice because of (mount + umount) vfs_node_load()
  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_down(node);// first countdown
  rwlock_unlock(&vfs_node_freelist.lock);

  if (node->n_count > 1)
    {
      printf("nfs_umount : FileSystem busy");
      return NFS_ERR;
    }
  else
    {
      rwlock_wrlock(&vfs_node_freelist.lock);
      vfs_node_down(node);// second countdown
      rwlock_unlock(&vfs_node_freelist.lock);

      // Get the private context field
      nfs_pv = node->n_ctx->ctx_pv;

      // from libnetwork
      net_nfs_umount(nfs_pv->server, mount_point);
      net_nfs_destroy(nfs_pv->server);

      // from VFS
      nfs_destroy_context(node->n_ctx);
    }

  return 0;
}
