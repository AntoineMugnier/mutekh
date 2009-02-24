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
#include "nfs.h"
#include "nfs-private.h"

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_INIT_NODE(nfs_init_node)
{
#ifdef CONFIG_NFS_DEBUG
  printk("nfs_init_node: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_RELEASE_NODE(nfs_release_node)
{
#ifdef CONFIG_NFS_DEBUG
  printk("nfs_release_node: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_node_s *parent
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_CREATE_NODE(nfs_create_node)
{
#ifdef CONFIG_NFS_DEBUG
  printk("nfs_create_node: you shoud not see that\n");
#endif

  return 0;
}

/*
** param	struct vfs_node_s *parent
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_LOOKUP_NODE(nfs_lookup_node)
{
/* error_t		nfs_lookup(struct nfs_s		*server, */
/* 			   const char		*path, */
/* 			   nfs_handle_t		directory, */
/* 			   nfs_handle_t		handle, */
/* 			   struct nfs_attr_s	*stat); */

  return NFS_OK;
}

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_WRITE_NODE(nfs_write_node)
{
#ifdef CONFIG_NFS_DEBUG
  printk("nfs_write_node: you shoud not see that\n");
#endif

    return 0;
}

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_UNLINK_NODE(nfs_unlink_node)
{
#ifdef CONFIG_NFS_DEBUG
  printk("nfs_unlink_node: you shoud not see that\n");
#endif

  return 0;
}
