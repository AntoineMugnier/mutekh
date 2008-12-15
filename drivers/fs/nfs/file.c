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
#include <netinet/nfs.h>
#include "nfs-private.h"


/*
** param	struct vfs_node_s *node
** param	struct vfs_file_s *file
** return	error_t
*/
VFS_OPEN_FILE(nfs_open)
{
/*   struct nfs_context_s	*ctx = NULL; */
/*   struct nfs_file_s	*file_pv = file->f_pv; */

/* #ifdef CONFIG_NFS_DEBUG */
/*   printf("nfs_open_file: trying to open %s\n", node->n_name); */
/* #endif */

/* /\*   // Is asking device node or /dev/\\* node? *\/ */
/* /\*   if (strcmp(node->n_name, "DEV")) *\/ */
/* /\*     { *\/ */
/*       // Getting Nfs context */
/*       ctx = nfs_get_ctx(); */

/*       // Is file/node existing? */
/*       if (nfs_lookup(ctx->server, PATH, file->handle, file->handle,) == NULL) */
/* 	return NFS_ERR; */
/* /\*     } *\/ */
/* /\* #ifdef CONFIG_NFS_DEBUG *\/ */
/* /\*   else *\/ */
/* /\*     printf("nfs_open_file: /dev is asked\n"); *\/ */
/* /\* #endif *\/ */

/*   // Allocating for a new Nfs file */
/*   if((file_pv = mem_alloc(sizeof(struct nfs_file_s), MEM_SCOPE_SYS)) == NULL) */
/*     return NFS_ERR; */

/*   // struct nfs_file_s only keep a struct vfs_node_s */
/*   // to access files */
/*   file_pv->node = node; */
/*   file_pv->current_node = NULL; */

/*   // relink nfs_file_s to vfs_file_s */
/*   file->f_pv = file_pv; */

  return NFS_OK;
}

/*
** param	struct vfs_file_s	*file
** param	uint8_t			*buffer
** param	size_t			size
** return	error_t
*/
/* VFS_READ_FILE(nfs_read) */
/* { */
/*   struct nfs_file_s	*file_pv = file->f_pv; */
/*   struct nfs_node_s	*node = file_pv->node->n_pv; */
/*   size_t		s = 0; */

/* #ifdef CONFIG_NFS_DEBUG */
/*   printf("nfs_read_file: starting reading %d bytes in %s\n", size, node->name); */
/* #endif */

/*   if (size == 0) */
/*     return 0; */

/*   switch(node->type) */
/*     { */
/*     case NFS_DIR : */
/*       break; */

/*     case NFS_CHAR : */
/*       if ((s = dev_char_read(node->device, buffer, size)) < 0) */
/* 	return EIO; */
/*       break; */

/*     case NFS_BLOCK : */
/* /\*       if ((s = dev_block_read(node->device, buffer, size)) < 0) *\/ */
/* /\*       	return EIO; *\/ */
/*       break; */

/*     default : */
/*       // Bad device type */
/*       return -NFS_ERR; */
/*     } */

/*   return s; */
/* } */

/*
** param	struct vfs_file_s	*file
** param	uint8_t			*buffer
** param	size_t			size
** return	error_t
*/
/* VFS_WRITE_FILE(nfs_write) */
/* { */
/*   struct nfs_file_s	*file_pv = file->f_pv; */
/*   struct nfs_node_s	*node = file_pv->node->n_pv; */
/*   size_t		s = 0; */

/* #ifdef CONFIG_NFS_DEBUG */
/*   printf("nfs_write_file: starting writing %d bytes in %s\n", size, node->name); */
/* #endif */

/*   if (size == 0) */
/*     return 0; */

/*   switch(node->type) */
/*     { */
/*     case NFS_DIR : */
/*       break; */

/*     case NFS_CHAR : */
/*       if ((s = dev_char_write(node->device, buffer, size)) < 0) */
/* 	return EIO; */
/*       break; */

/*     case NFS_BLOCK : */
/* /\*       if ((s = dev_block_write (node->device, buffer, size)) < 0) *\/ */
/* /\* 	return EIO; *\/ */
/*       break; */

/*     default : */
/*       // Bad device type */
/*       return -NFS_ERR; */
/*     } */

/*   return s; */
/* } */

/*
** param	struct vfs_file_s	*file
** return	error_t
*/
VFS_LSEEK_FILE(nfs_lseek)
{
#ifdef CONFIG_NFS_DEBUG
  printf("nfs_lseek_file: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_file_s	*file
** return	error_t
*/
VFS_RELEASE_FILE(nfs_release)
{
#ifdef CONFIG_NFS_DEBUG
  printf("nfs_release_file: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_file_s	*file
** param	struct vfs_dirent_s	*dirent
** return	error_t
*/
/* VFS_READ_DIR(nfs_readdir) */
/* { */
/*   struct nfs_context_s	*ctx = NULL; */
/*   struct nfs_file_s		*file_pv = file->f_pv; */

/* #ifdef CONFIG_NFS_DEBUG */
/*   printf("nfs_read_dir: reading directory %s\n", file_pv->node->n_name); */
/*   printf("nfs_read_dir: from file %s\n", dirent->d_name); */
/* #endif */

/*   ctx = nfs_get_ctx(); */

/*   // Is that the first entering for dirent? */
/*   if (file_pv->current_node == NULL) */
/*     { */
/*       // get the head of the hash table */
/*       if ((file_pv->current_node = nfs_hashfunc_head(&(ctx->hash))) == NULL) */
/* 	return NFS_DIREMPTY; // dir is empty */

/*       // Fill fields */
/*       strncpy(dirent->d_name, file_pv->current_node->name, VFS_MAX_NAME_LENGTH); */
/*       dirent->d_size = 0; */
/*       dirent->d_type = file_pv->current_node->type; */

/*       return NFS_OK; */
/*     } */
/*   else */
/*     // Get next node from the hash table */
/*     if ((file_pv->current_node = nfs_hashfunc_next(&(ctx->hash), file_pv->current_node)) == NULL) */
/*       return NFS_DIREMPTY; // End of dir */
/*     else */
/*       { */
/* 	strncpy(dirent->d_name, file_pv->current_node->name, VFS_MAX_NAME_LENGTH); */
/* 	dirent->d_size = 0; */
/* 	dirent->d_type = file_pv->current_node->type; */

/* 	return NFS_OK; */
/*       } */
/* } */
