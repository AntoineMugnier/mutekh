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
#include <drivers/fs/devfs/devfs.h>

error_t	devfs_register(char*			name,
		       enum devfs_type_e	type)
{
  return 0;
}

error_t	devfs_unregister(char*	name)
{
  return 0;
}

error_t	devfs_init(struct vfs_node_s	*root)
{
  struct vfs_context_s	*devfs_ctx;
  struct vfs_node_s	*dev_node;
  struct vfs_node_s	*new_node;
  uint_fast32_t		flags;
  //  char			*dirs_ptr[1] = "dev";
  error_t		err;
  bool_t		isAbsolutePath = 0;

  VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL);

  // Allocating memory for parent context
  if((devfs_ctx = mem_alloc(sizeof(*devfs_ctx), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  // Initializing context for DevFS (/dev)
  if ((err = vfs_node_load(root, "dev", flags, isAbsolutePath, &dev_node)))
    return err;

  devfs_ctx->ctx_type = VFS_DEVICE_TYPE;
  devfs_ctx->ctx_dev = NULL;
  devfs_ctx->ctx_op = (struct vfs_context_op_s *) &devfs_ctx_op;
  devfs_ctx->ctx_node_op = (struct vfs_node_op_s *) &devfs_n_op;
  devfs_ctx->ctx_file_op = (struct vfs_file_op_s *) &devfs_f_op;

  // Set private field in vfs_context_s
  devfs_ctx_op.create(devfs_ctx);

  ///////////////////
  // Creating Node //
  ///////////////////

  // Get node from the freelist
  new_node = vfs_node_freelist_get(devfs_ctx);

  // Naming the new node
  strcpy(new_node->n_name, "tty");

  // Add node in children list "n_children" in parent's node
  vfs_freeList_push(&dev_node->n_children, &new_node);

  // Fullfill node_pv (also n_attr in VFS (optionnale))
  new_node->n_op->create(dev_node, new_node);

  ///////////////////


/* VFS_NODE_LOAD(n) */
/*   error_t (n) (struct vfs_node_s *root,	\ */
/* 	       char **path,		\ */
/* 	       uint_fast32_t flags,	\ */
/* 	       bool_t isAbsolutePath,	\ */
/* 	       struct vfs_node_s **node) */

  // Creating new needed nodes
  vfs_node_load(root, "zero", flags, 0, &dev_node);
  vfs_node_load(root, "null", flags, 0, &dev_node);

  return 0;
}
