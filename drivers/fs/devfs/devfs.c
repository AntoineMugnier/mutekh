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


static inline error_t devfs_new_node(struct vfs_node_s	*dev_node,
				     char*		name,
				     uint_fast32_t	flags)
{
  struct vfs_node_s	*new_node;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_new_node: Creating Node %s\n", name);
#endif

  // Get node from the freelist
  if ((new_node = vfs_node_freelist_get(dev_node->n_ctx)) == NULL)
    return VFS_EUNKNOWN;

  // Naming the new node
  strcpy(new_node->n_name, name);

  // Add node in children list "n_children" in parent's node
  vfs_node_list_push(&dev_node->n_children, &new_node);

  // Fullfill node_pv (also n_attr in VFS (optionnale))
  new_node->n_op->create(dev_node, new_node);

  return 0;
}

  /* VFS_NODE_LOAD(n) */
  /*   error_t (n) (struct vfs_node_s *root,	\ */
  /* 	       char **path,		\ */
  /* 	       uint_fast32_t flags,	\ */
  /* 	       bool_t isAbsolutePath,	\ */
  /* 	       struct vfs_node_s **node) */


error_t	devfs_init(struct vfs_node_s	*root)
{
  struct vfs_context_s	*devfs_ctx;
  struct vfs_node_s	*dev_node;
  struct vfs_node_s	*new_node;
  char			*dev_name;
  error_t		err;
  uint_fast32_t		flags;
  bool_t		isAbsolutePath = 0;

  //////////////////////////////////////////////////

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: Setting up /dev Node\n");
  printf("devfs_init: Initializing context for DevFS (/dev)\n");
#endif

  // Allocating memory for parent context
  if((devfs_ctx = mem_alloc(sizeof(*devfs_ctx), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;
  // Allocating memory for /dev name
  if((dev_name = mem_alloc(sizeof(char) * 4, MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  // Naming the new node
  strcpy(dev_name, "dev");

  // Setting up flags
  VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL);

  printf("dev_name is : %s\n", dev_name);
  printf("dev_node is : %p\n", dev_node);

  if ((err = vfs_node_load(root, &dev_name, flags, isAbsolutePath, &dev_node)))
    return err;

  printf("--\n PASSING IT\n--\n");

  devfs_ctx->ctx_type = VFS_DEVICE_TYPE;
  devfs_ctx->ctx_dev = NULL;
  devfs_ctx->ctx_op = (struct vfs_context_op_s *) &devfs_ctx_op;
  devfs_ctx->ctx_node_op = (struct vfs_node_op_s *) &devfs_n_op;
  devfs_ctx->ctx_file_op = (struct vfs_file_op_s *) &devfs_f_op;

  // Set private field in vfs_context_s
  devfs_ctx_op.create(devfs_ctx);

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: Creating Node\n");
#endif

  //////////////////////////////////////////////////

  devfs_new_node(dev_node, flags, "tty");

  //////////////////////////////////////////////////

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: DevFS Initialized\n");
#endif

  return 0;
}
