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
#include <vfs/vfs-private.h>
#include <netinet/nfs.h>

// Return code
#define NFS_OK		0
#define NFS_ERR		1
#define NFS_DIREMPTY	14 // to match VFS_EODIR

struct nfs_node_s
{
};

struct nfs_context_s
{
  struct nfs_s	*server;
  nfs_handle_t	root;
};

struct nfs_file_s
{
  struct vfs_node_s	*node;
  struct nfs_attr_s	*stat;
  nfs_handle_t		handle;
};

////////////////////////////////////////////////////

/* // Used to get Nfs context from anywhere */
/* static inline struct nfs_context_s	*nfs_get_ctx(const char	*mount_point) */
/* { */
/*   struct vfs_node_s		*dev_node = NULL; */

/*   //get node to acces n_ctx field */
/*   if ((dev_node = vfs_node_lookup(vfs_get_root(), (char*) mount_point)) == NULL) */
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
