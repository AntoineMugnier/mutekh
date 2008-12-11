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

#ifndef __NFS_PRIVATE_H__
#define __NFS_PRIVATE_H__

#include <hexo/types.h>
#include <vfs/vfs.h>
#include <gpct/cont_hashlist.h>
#include <vfs/vfs-private.h>

// Return code
#define NFS_OK		0
#define NFS_ERR		1
#define NFS_DIREMPTY		14 // to match VFS_EODIR


// Values for  nfs_node_s->type;
/* #define NFS_DIR		0x00 */


struct nfs_node_s
{
/*   CONTAINER_ENTRY_TYPE(HASHLIST)        hash_entry; */
/*   struct device_s			*device; */
/*   const char				*name; */
/*   uint_fast8_t				type; */
};

/* CONTAINER_TYPE    (nfs_hash, HASHLIST, struct nfs_node_s, hash_entry, 111); */
/* CONTAINER_KEY_TYPE(nfs_hash, STRING, name); */

/* CONTAINER_FUNC    (nfs_hash, HASHLIST, static, nfs_hashfunc, name); */
/* CONTAINER_KEY_FUNC(nfs_hash, HASHLIST, static, nfs_hashfunc, name); */

struct nfs_context_s
{
/*   nfs_hash_root_t	hash; */
};

struct nfs_file_s
{
/*   struct vfs_node_s	*node; */
/*   struct nfs_node_s	*current_node; // used for readdir() */
};

////////////////////////////////////////////////////

/* // Used to get Nfs context from anywhere */
/* static inline struct nfs_context_s	*nfs_get_ctx() */
/* { */
/*   struct vfs_node_s		*dev_node = NULL; */

/*   //get node to acces n_ctx field */
/*   if ((dev_node = vfs_node_lookup(vfs_get_root(), "DEV")) == NULL) */
/*     return NULL; */

/*   return (dev_node->n_ctx->ctx_pv); */
/* } */

/* //////////////////////////////////////////////////// */

/* // Used to get Nfs node from anywhere */
/* static inline struct vfs_node_s	*nfs_get_node() */
/* { */
/*   struct vfs_node_s		*dev_node = NULL; */
/*   char				*t = NULL; */

/*   if ((t = strrchr(NFS_MOUNT_POINT, '/'))) */
/*     {   */
/*       if ((dev_node = vfs_node_lookup(vfs_get_root(), ++t)) == NULL) */
/* 	return NULL; */
/*       else */
/* 	return (dev_node); */
/*     } */
/*   else */
/*     { */
/*       //get node to acces n_ctx field */
/*       if ((dev_node = vfs_node_lookup(vfs_get_root(), "DEV")) == NULL) */
/* 	return NULL; */
/*       else */
/* 	return (dev_node); */
/*     } */
/* } */

////////////////////////////////////////////////////

#endif /* __NFS_PRIVATE_H__ */
