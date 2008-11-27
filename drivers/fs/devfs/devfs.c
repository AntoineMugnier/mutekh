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

/* static inline char	*devfs_get_name(const char*	name) */
/* { */
/*   char	*node_name; */
/*   int	i; */
/*   int	j = 0; */

/*   assert(name != NULL); */

/*   // go to end of line */
/*   for (i = 0; name[i] != '\0'; ++i) */
/*     if (name[i] == '/' && name[i + 1] != '\0') */
/*       j = i; */

/*   // count last name's size */
/*   for (i = 1; name[j + i] != '\0' && name[j + i] != '/'; ++i) */
/*     continue; */

/*   if ((node_name = mem_alloc(sizeof(char) * (i + 1), MEM_SCOPE_SYS)) == NULL) */
/*     return NULL; */

/*   strncpy(node_name, &(name[j + 1]), i); */
/*   node_name[i + 1] = '\0'; */

/*   return node_name; */
/* } */

////////////////////////////////////////////////////////

error_t	devfs_init(struct vfs_node_s	*root,
		   const char		*mount_point)
{
  struct vfs_context_s		*vfs_ctx = NULL;
  error_t			err = 0;
  uint_fast32_t			flags = 0;
  struct vfs_file_s		*file = NULL;

  assert(mount_point != NULL);
  assert(root != NULL);

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: Mounting DevFS in %s\n", mount_point);
#endif

  // Some check
  if (mount_point[0] != '/')
    {
      printf("devfs_init: mount_point must be absolute, abording\n");
      return DEVFS_ERR;
    }

  // Allocating memory for parent context
  if((vfs_ctx = mem_alloc(sizeof(struct vfs_context_s), MEM_SCOPE_SYS)) == NULL)
    return -VFS_ENOMEM;

  // Setting up flags
  VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_EXCL | VFS_DIR);

  // Checking the mount_point (/dev) node
  if ((err = vfs_open(root, mount_point, flags, (uint_fast16_t) flags, &file)))
    {
      printf("devfs_init: %s doesn't seem to exist in filesystem, abording\n", mount_point);
      return err;
    }
  vfs_close(file);

/*   if ((err = vfs_node_load(root, mount_point, flags, isAbsolutePath, &dev_node))) */
/*     { */
/*       printf("/dev does not exist in root filesystem, abording\n"); */
/*       return err; */
/*     } */

  // Setting up vfs_context_s
  vfs_ctx->ctx_type = VFS_DEVICE_TYPE;
  vfs_ctx->ctx_dev = NULL;
  vfs_ctx->ctx_op = (struct vfs_context_op_s *) &devfs_ctx_op;
  vfs_ctx->ctx_node_op = (struct vfs_node_op_s *) &devfs_n_op;
  vfs_ctx->ctx_file_op = (struct vfs_file_op_s *) &devfs_f_op;

  vfs_ctx->ctx_op->create(vfs_ctx);

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_init: DevFS Initialized\n");
#endif

  return DEVFS_OK;
}

////////////////////////////////////////////////////////

struct devfs_node_s *devfs_register(struct devfs_context_s	*ctx,
				    const char			*name,
				    struct device_s		*device,
				    uint_fast8_t		type)
{
  uint_fast32_t		flags;
  struct devfs_node_s	*new_node;
  struct vfs_file_s	*file;
  char			*path_name;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_register: Registering device node %s\n", name);
#endif

  // Does node already exist?
  if (devfs_hashfunc_lookup(&(ctx->hash), name) != NULL)
    return NULL;

  // Setting up flags for VFS and checking if asked devide node
  // type is an existing one
  switch(type)
    {
    case DEVFS_DIR :
      VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_CREATE | VFS_O_EXCL | VFS_DIR);
      break;

    case DEVFS_CHAR :
    case DEVFS_BLOCK :
      VFS_SET(flags, VFS_O_DIRECTORY | VFS_O_CREATE | VFS_O_EXCL | VFS_DEVICE);
      break;

    default :
      // Bad device type
      return NULL;
    }

  // Allocating for a new DevFS node
  if((new_node = mem_alloc(sizeof(struct devfs_node_s), MEM_SCOPE_SYS)) == NULL)
    return NULL;

  // Setting up the new node
  new_node->name = name;
  new_node->type = type;
  new_node->device = device;

  // Adding node to hash list
  devfs_hashfunc_push(&(ctx->hash), new_node);

  // Adding VFS node
  if((path_name = mem_alloc(sizeof(char) * (strlen(name) + 6), MEM_SCOPE_SYS)) == NULL)
    return NULL;

  strncpy(path_name, "/dev/", 6);
  strcat(path_name, name);

  ////
  //// Need to be checked because NULL is passed
  //// whereas struct vfs_node_s* root should be
  ////
  if ((vfs_open(NULL, path_name, flags, (uint_fast16_t) flags, &file)))
    return NULL;

  vfs_close(file);

  /* devfs_init_node(vfs_node); */
/*   devfs_create_node(dev_node, new_node); */


  return new_node;
}

////////////////////////////////////////////////////////

error_t	devfs_unregister(struct devfs_context_s	*ctx,
			 struct devfs_node_s	*dnode)
{

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_unregister: Unregistering device node %s\n", dnode->name);
#endif

  // Does node already exist?
  if (devfs_hashfunc_lookup(&(ctx->hash), dnode->name) != NULL)
    return DEVFS_ERR;

  // Removing node from hash list
  devfs_hashfunc_remove(&(ctx->hash), dnode->name);

  // Cleaning up the mess
  devfs_release_node( (struct vfs_node_s*) dnode);
  devfs_unlink_node( (struct vfs_node_s*) dnode);

  return DEVFS_OK;
}

error_t	devfs_destroy(struct devfs_context_s	*ctx)
{

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_destroy: Destroying devFS context \n");
#endif

  devfs_hashfunc_destroy(&(ctx->hash));

  mem_free(ctx);

  return DEVFS_OK;
}
