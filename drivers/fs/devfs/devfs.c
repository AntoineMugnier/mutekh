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
#include <assert.h>


////////////////////////////////////////////////////////

error_t	devfs_init(const char		*mount_point)
{
  struct vfs_context_s		*devfs_ctx = NULL;
  struct vfs_node_s		*dev_node = NULL;
  error_t			err = 0;
  uint_fast32_t			flags = 0;
  char				*dirs_ptr[vfs_dir_count(mount_point) + 1];
  bool_t			isAbsolutePath = 1;

  assert(mount_point != NULL);

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_init: Mounting DevFS in %s\n", mount_point);
#endif

  // Some check
  if (mount_point[0] != '/')
    {
      printf("devfs_init: mount_point must be absolute, abording\n");
      return DEVFS_ERR;
    }

  // translate from char* to char**
  vfs_split_path(mount_point, dirs_ptr);

  // Setting up flags
  VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_EXCL | VFS_DIR);

  // Get the node if existing
  if((err = vfs_node_load(vfs_get_root(), dirs_ptr, flags, isAbsolutePath, &dev_node)))
    {
      printf("devfs_init: %s doesn't seem to exist in filesystem, abording\n", mount_point);
      return err;
    }

/*   // Checking the mount_point (/dev) node */
/*   if ((err = vfs_open(vfs_get_root(), mount_point, flags, (uint_fast16_t) flags, &file))) */
/*     { */
/*       printf("devfs_init: %s doesn't seem to exist in filesystem, abording\n", mount_point); */
/*       return err; */
/*     } */
/*   vfs_close(file); */

  // Allocating memory for parent context
  if((devfs_ctx = mem_alloc(sizeof(struct vfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  // Setting up vfs_context_s
  devfs_ctx->ctx_type = VFS_DEVICE_TYPE;
  devfs_ctx->ctx_dev = NULL;
  devfs_ctx->ctx_op = (struct vfs_context_op_s *) &devfs_ctx_op;
  devfs_ctx->ctx_node_op = (struct vfs_node_op_s *) &devfs_n_op;
  devfs_ctx->ctx_file_op = (struct vfs_file_op_s *) &devfs_f_op;
  devfs_ctx->ctx_pv = NULL;

  // create intern stuff
  if (devfs_ctx->ctx_op->create(devfs_ctx))
    return DEVFS_ERR;

/*   // get /dev node from root filesystem */
/*   dev_node = devfs_get_node(); */
  
/*   if (dev_node == NULL) */
/*     printf("devfs_init: lookup failed\n"); */
  

  // change context type of mount_point to DevFS
  dev_node->n_ctx = devfs_ctx;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_init: DevFS Initialized\n");
#endif

  return DEVFS_OK;
}

////////////////////////////////////////////////////////

struct devfs_node_s *devfs_register(const char			*name,
				    struct device_s		*device,
				    uint_fast8_t		type)
{
  struct devfs_context_s	*ctx = NULL;
  struct devfs_node_s		*new_node = NULL;
  /*   struct vfs_node_s		*dev_node = NULL; */
  /*   struct vfs_file_s		*file = NULL; */
  /*   char				*path_name = NULL; */
  /*   uint_fast32_t			flags = 0; */

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_register: Registering device node %s\n", name);
#endif

  if ((ctx = devfs_get_ctx()) == NULL)
    {
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
      printf("devfs_register: Could not get devfs context\n");
#endif
      return NULL;
    }

  // Does node already exist?
  if (devfs_hashfunc_lookup(&(ctx->hash), name) != NULL)
    {
      printf("devfs_register: %s already in use, please give it another name\n", name);
      return NULL;
    }

  /*   // Setting up flags for VFS and checking if asked device node */
  /*   // type is an existing one */
  /*   switch(type) */
  /*     { */
  /*     case DEVFS_DIR : */
  /*       VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_CREATE | VFS_O_EXCL | VFS_DIR); */
  /*       break; */

  /*     case DEVFS_CHAR : */
  /*     case DEVFS_BLOCK : */
  /*       VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_CREATE | VFS_O_EXCL | VFS_DEVICE); */
  /*       break; */

  /*     default : */
  /*       // Bad device type */
  /*       return NULL; */
  /*     } */

  // Allocating for a new DevFS node
  if((new_node = mem_alloc(sizeof(struct devfs_node_s), MEM_SCOPE_SYS)) == NULL)
    return NULL;

  // Setting up the new node
  new_node->name = name;
  new_node->type = type;
  new_node->device = device;

  // Adding node to hash list
  if ((devfs_hashfunc_push(&(ctx->hash), new_node)) == 0)
    {
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
      printf("devfs_register: Could not push %s in hash table. Check Hash table.\n");
#endif
      return NULL;
    }

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
/*   CONTAINER_FOREACH(devfs_hash, HASHLIST, &ctx->hash,{ */
/*     printf("devfs_register: inside hash table is node %s\n", item->name); */
/*   }); */
#endif

  return new_node;
}

////////////////////////////////////////////////////////

error_t	devfs_unregister(struct devfs_context_s	*ctx,
			 struct devfs_node_s	*dnode)
{

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_unregister: Unregistering device node %s\n", dnode->name);
#endif

  // Does node already exist?
  if (devfs_hashfunc_lookup(&(ctx->hash), dnode->name) != NULL)
    return DEVFS_ERR;

  // Removing node from hash list
/*   devfs_hashfunc_remove(&(ctx->hash), dnode->name); */

  // Cleaning up the mess
  devfs_release_node( (struct vfs_node_s*) dnode);
  devfs_unlink_node( (struct vfs_node_s*) dnode);

  return DEVFS_OK;
}

////////////////////////////////////////////////////////

error_t	devfs_destroy(struct devfs_context_s	*ctx)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_destroy: Destroying devFS context \n");
#endif

  devfs_hashfunc_destroy(&(ctx->hash));

  mem_free(ctx);

  return DEVFS_OK;
}
