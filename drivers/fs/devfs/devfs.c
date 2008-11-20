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
#include <drivers/fs/devfs/devfs-private.h>


static inline error_t devfs_new_node(struct vfs_node_s	*dev_node,
				     char*		name,
				     uint_fast32_t	flags)
{
  //////////////////////////////////////////////
  //  struct vfs_node_s	*dev_node;


#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: Creating Nodes\n");
#endif

  VFS_CLEAR(flags, 0);
  VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_DEVICE);

  devfs_new_node(dev_node, "tty", flags);

  VFS_CLEAR(flags, 0);
  VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_DEVICE);

  devfs_new_node(dev_node, "null", flags);
  devfs_new_node(dev_node, "zero", flags);
  devfs_new_node(dev_node, "random", flags);
  devfs_new_node(dev_node, "urandom", flags);
  //////////////////////////////////////////////


/*   if (vfs_node_lookup(dev_node, name)) */
/*     return VFS_EEXIST; */

  // Get node from the freelist
/*   if ((new_node = vfs_node_freelist_get(dev_node->n_ctx)) == NULL) */
/*     return VFS_EUNKNOWN; */

  // Naming the new node
/*   strcpy(new_node->n_name, name); */

/*   VFS_SET(new_node->n_flags, flags); */
/*   VFS_SET(new_node->n_attr, flags); */

  // Add node in children list "n_children" in parent's node
/*   vfs_node_list_push(&dev_node->n_children, new_node); */

  // Fulfill node_pv (also n_attr in VFS (optional))
/*   new_node->n_op->init(new_node); */
/*   new_node->n_op->create(dev_node, new_node); */

  return 0;
}

struct devfs_node_s *devfs_register(struct devfs_context_s	*ctx,
				    const char			*name,
				    struct device_s		*device,
				    uint_fast8_t			type)
{
  uint_fast32_t		flags;
  struct devfs_node_s	*new_node;
  struct vfs_node_s	*vfs_node;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_register: Registering device node %s\n", name);
#endif

  // Initialization for DevFS node
  if((new_node = mem_alloc(sizeof(struct devfs_node_s), MEM_SCOPE_SYS)) == NULL)
    return NULL;

  new_node->name = name;
  new_node->type = type;
  new_node->device = device;

  //  devfs_hash_push(&(ctx->hash), new_node);

  // Initialization for VFS node
  if((vfs_node = mem_alloc(sizeof(struct vfs_node_s), MEM_SCOPE_SYS)) == NULL)
    return NULL;
  
  devfs_init_node(vfs_node);
  //devfs_create_node(dev_node, new_node);

/*   // Does node already exist? */
/*   if (vfs_node_lookup(vfs_dev_node, name)) */
/*     return VFS_EEXIST; */

  // Setting up flags
  switch(type)
    {
    case DEVFS_DIR :
      VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_DIR);

      break;
    case DEVFS_CHAR :
      VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_DEVICE);

      break;
    case DEVFS_BLOCK :
      VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_DEVICE);

      break;
/*     case DEVFS_PSEUDO : */
/*       VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_DEV_PSEUDO); */

/*       break; */
    default :
      // Bad device type
      return NULL;
    }

  ////////////////////////////////////////////////::


  //  devfs_new_node(vfs_dev_node, name, flags);

  return new_node;
}

error_t	devfs_unregister(struct devfs_node_s	*dnode)
{
  struct vfs_node_s	*del_node;

  // Does node exist?
  //  if ((del_node = vfs_node_lookup(vfs_dev_node, name)) == NULL)
  //      return -VFS_EEXIST;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_unregister: Unregistering device node %s\n", dnode->name);
#endif

  del_node->n_op->release(del_node);
  del_node->n_op->unlink(del_node);

  return 0;
}

struct devfs_context_s	*devfs_init()
{
  struct devfs_context_s	*ctx;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: Initializing DevFS context\n");
#endif

  if((ctx = mem_alloc(sizeof(struct devfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return NULL;

  //  devfs_hash_init(&(ctx->hash));
  
  return ctx;
}

error_t	devfs_mount(struct devfs_context_s	*ctx,
		    const char			*mount_point)
{
  struct vfs_context_s	*devfs_ctx;
/*   const char		*dev_name[2] = { mount_point, NULL}; */
/*   error_t		err; */
  uint_fast32_t		flags;
/*   bool_t		isAbsolutePath = 0; */

  //////////////////////////////////////////////////

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: Mounting DevFS (%s)\n", mount_point);
#endif

  // Allocating memory for parent context
  if((devfs_ctx = mem_alloc(sizeof(struct vfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;
 
  // Setting up flags
  VFS_SET(flags, VFS_DIR);

  // Creating the /dev node
/*   if ((err = vfs_open())) */
/*     { */
/*       printf("/dev does not exist in root filesystem, abording\n"); */
/*       return err; */
/*     } */

  /*     if ((err = vfs_node_load(root, dev_name, flags, isAbsolutePath, &dev_node))) */
  /*     { */
  /*       printf("/dev does not exist in root filesystem, abording\n"); */
  /*       return err; */
  /*     } */

  // Setting up vfs_context_s
  devfs_ctx->ctx_type = VFS_DEVICE_TYPE;
  devfs_ctx->ctx_dev = NULL;
  devfs_ctx->ctx_op = (struct vfs_context_op_s *) &devfs_ctx_op;
  devfs_ctx->ctx_node_op = (struct vfs_node_op_s *) &devfs_n_op;
  devfs_ctx->ctx_file_op = (struct vfs_file_op_s *) &devfs_f_op;

  // Set private field in vfs_context_s
  devfs_ctx->ctx_pv = ctx;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: DevFS Initialized\n");
#endif

  return 0;
}
