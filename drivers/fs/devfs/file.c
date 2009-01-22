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
#include <device/char.h>
/* #include <device/block.h> */
#include <device/driver.h>

#include "devfs-private.h"
#include "devfs.h"


/*
** param	struct vfs_node_s *node
** param	struct vfs_file_s *file
** return	error_t
*/
VFS_OPEN_FILE(devfs_open)
{
  struct devfs_context_s	*ctx = NULL;
  struct devfs_file_s		*file_pv = file->f_pv;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_open_file: trying to open %s\n", node->n_name);
#endif

  // Is asking device node or /dev/* node?
  if (node != devfs_get_root_node())
    {
      // Getting DevFS context
      ctx = devfs_get_ctx();

      // Is file/node existing?
      if (devfs_hashfunc_lookup(&(ctx->hash), node->n_name) == NULL)
	return DEVFS_ERR;
    }
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  else
    printf("devfs_open_file: DevFS root is asked\n");
#endif

  // Allocating for a new DevFS file
  if((file_pv = mem_alloc(sizeof(struct devfs_file_s), MEM_SCOPE_SYS)) == NULL)
    return DEVFS_ERR;

  // struct devfs_file_s only keep a struct vfs_node_s
  // to access files
  file_pv->node = node;
  file_pv->current_node = NULL;

  // relink devfs_file_s to vfs_file_s
  file->f_pv = file_pv;

  // Setting up flags
  VFS_SET(file->f_flags, VFS_O_DEVICE);

  return DEVFS_OK;
}

/*
** param	struct vfs_file_s	*file
** param	uint8_t			*buffer
** param	size_t			size
** return	error_t
*/
VFS_READ_FILE(devfs_read)
{
  struct devfs_file_s		*file_pv = file->f_pv;
  struct devfs_node_s		*node_pv = file_pv->node->n_pv;
  struct devfs_context_s	*ctx = NULL;
  size_t			s = 0;

  // Getting DevFS context
  ctx = devfs_get_ctx();

  // Getting back node's private field
  if ((node_pv = devfs_hashfunc_lookup(&(ctx->hash), file_pv->node->n_name)) == NULL)
    return -DEVFS_ERR;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_read_file: starting reading %d bytes in %s\n", size, file_pv->node->n_name);
#endif

  if (size == 0)
    return 0;

  switch(node_pv->type)
    {
    case DEVFS_DIR :
      break;

    case DEVFS_CHAR :
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
      printf("trying to read on char device\n");
#endif
      if ((s = dev_char_wait_read(node_pv->device, buffer, size)) < 0)
	return EIO;
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
      printf("devfs_read : size returned : %d\n", s);
#endif
      break;

    case DEVFS_BLOCK :
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
      printf("trying to read on char device\n");
      printf("file->f_offset value : %d\n", file->f_offset);
#endif
/*       if ((s = dev_block_wait_read(node_pv->device, buffer, file->f_offset, size)) < 0) */
/* 	return EIO; */
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
      printf("devfs_read : size returned : %d\n", s);
#endif
      break;

    default :
      // Bad device type
      return -DEVFS_ERR;
    }

  return s;
}

/*
** param	struct vfs_file_s	*file
** param	uint8_t			*buffer
** param	size_t			size
** return	error_t
*/
VFS_WRITE_FILE(devfs_write)
{
  struct devfs_file_s		*file_pv = file->f_pv;
  struct devfs_node_s		*node_pv = file_pv->node->n_pv;
  struct devfs_context_s	*ctx = NULL;
  size_t			s = 0;

  // Getting DevFS context
  ctx = devfs_get_ctx();

  // Getting back node's private field
  if ((node_pv = devfs_hashfunc_lookup(&(ctx->hash), file_pv->node->n_name)) == NULL)
    return -DEVFS_ERR;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_write_file: starting writing %d bytes in %s\n", size, file_pv->node->n_name);
#endif

  if (size == 0)
    return 0;

  switch(node_pv->type)
    {
    case DEVFS_DIR :
      break;

    case DEVFS_CHAR :
      if ((s = dev_char_wait_write(node_pv->device, buffer, size)) < 0)
	return EIO;
      break;

    case DEVFS_BLOCK :
      if ((s = dev_block_wait_write (node_pv->device, buffer, file->f_offset, size)) < 0)
	return EIO;
      break;

    default :
      // Bad device type
      return -DEVFS_ERR;
    }

  return s;
}

/*
** param	struct vfs_file_s	*file
** return	error_t
*/
VFS_LSEEK_FILE(devfs_lseek)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_lseek_file: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_file_s	*file
** return	error_t
*/
VFS_RELEASE_FILE(devfs_release)
{
#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_release_file: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_file_s	*file
** param	struct vfs_dirent_s	*dirent
** return	error_t
*/
VFS_READ_DIR(devfs_readdir)
{
  struct devfs_context_s	*ctx = NULL;
  struct devfs_file_s		*file_pv = file->f_pv;

#ifdef CONFIG_DRIVER_FS_DEVFS_DEBUG
  printf("devfs_read_dir: reading directory %s\n", file_pv->node->n_name);
#endif

  ctx = devfs_get_ctx();

  // Is that the first entering for dirent?
  if (file_pv->current_node == NULL)
    {
      // get the head of the hash table
      if ((file_pv->current_node = devfs_hashfunc_head(&(ctx->hash))) == NULL)
	return DEVFS_DIREMPTY; // dir is empty

      // Fill fields
      strncpy(dirent->d_name, file_pv->current_node->name, VFS_MAX_NAME_LENGTH);
      dirent->d_size = 0;
      dirent->d_type = file_pv->current_node->type;

      return DEVFS_OK;
    }
  else
    // Get next node from the hash table
    if ((file_pv->current_node = devfs_hashfunc_next(&(ctx->hash), file_pv->current_node)) == NULL)
      {
	memset(dirent, '\0', sizeof(struct vfs_dirent_s));
	return DEVFS_DIREMPTY; // End of dir
      }
    else
      {
	strncpy(dirent->d_name, file_pv->current_node->name, VFS_MAX_NAME_LENGTH);
	dirent->d_size = 0;
	dirent->d_type = file_pv->current_node->type;

	return DEVFS_OK;
      }
}
