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

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_open_file: trying to open %s\n", node->n_name);
#endif

  // Is asking device node or /dev/* node?
  if (strcmp(node->n_name, "DEV"))
    {
      // Getting DevFS context
      ctx = devfs_get_ctx();

      // Is file/node existing?
      if (devfs_hashfunc_lookup(&(ctx->hash), node->n_name) == NULL)
	return DEVFS_ERR;
    }

  // Allocating for a new DevFS file
  if((file_pv = mem_alloc(sizeof(struct devfs_file_s), MEM_SCOPE_SYS)) == NULL)
    return DEVFS_ERR;

  // struct devfs_file_s only keep a struct vfs_node_s
  // to access files
  file_pv->node = node;

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
  size_t	s = 0;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_read_file: you should not see that\n");
#endif

  /* #if DEVFS_DEBUG */
  /*     printf("++++ devfs_read started, asked size %d\n", size); */
  /* #endif */

  /*     if (file->f_node->n_attr & VFS_FIFO) */
  /* 	return -EINVAL; */

  /*     if (size == 0) */
  /* 	return 0; */

  /*     if (node->n_pv->type == DEVFS_CHAR) */
  /*     { */
  /* #if DEVFS_DEBUG */
  /* 	printf("++++ devfs_read on type char\n"); */
  /* #endif */
  /* 	if ((s = dev_char_read(*(struct device) file, buffer, size)) < 0) */
  /* 	    return EIO; */
  /*     } */
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
    size_t	s = 0;

#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_write_file: you should not see that\n");
#endif

/* #if DEVFS_DEBUG */
/*     printf("++++ devfs_write started, asked size %d\n", size); */
/* #endif */

/*     if (node->n_pv->type == DEVFS_CHAR) */
/*     { */
/* #if DEVFS_DEBUG */
/* 	printf("++++ devfs_write on type char\n"); */
/* #endif */
/* 	if ((s = dev_char_write(*(struct device) file, buffer, size)) < 0) */
/* 	    return EIO; */
/*     } */
    return s;
}

/*
** param	struct vfs_file_s	*file
** return	error_t
*/
VFS_LSEEK_FILE(devfs_lseek)
{
#ifdef CONFIG_DEVFS_DEBUG
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
#ifdef CONFIG_DEVFS_DEBUG
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
  struct devfs_node_s		*new_node = NULL;
  struct devfs_file_s		*file_pv = file->f_pv;
  const char			*cur_name;


#ifdef CONFIG_DEVFS_DEBUG
  printf("devfs_read_dir: reading directory %s\n", file_pv->node->n_name);
#endif

  ctx = devfs_get_ctx();

  // Is that the first entering for dirent?
  if ((devfs_hashfunc_lookup(&(ctx->hash), dirent->d_name)) == NULL)
    {
      // Get the head
      // NOT WORKING
      if ((cur_name = devfs_hashfunc_head(&(ctx->hash))) == NULL)
	{
#ifdef CONFIG_DEVFS_DEBUG
	  printf("devfs_read_dir: HERE in %s\n", dirent->d_name);
#endif
	  return DEVFS_ERR;
	}

      strcpy(dirent->d_name, cur_name);

      // Fill fields
      new_node = devfs_hashfunc_lookup(&(ctx->hash), dirent->d_name);
      dirent->d_size = 666; // Change it to what you need/want
      dirent->d_type = new_node->type;

      return DEVFS_OK;
    }
  else
    // Get next node from the hash table
    if ((cur_name = devfs_hashfunc_next(&(ctx->hash), dirent->d_name)) == NULL)
      return DEVFS_ERR;
    else
      {
	strcpy(dirent->d_name, cur_name);
	return DEVFS_OK;
      }
}
