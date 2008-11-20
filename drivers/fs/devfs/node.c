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

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_INIT_NODE(devfs_init_node)
{
  struct devfs_node_s *node_pv;

  /*     if(node->n_pv != NULL) */
  /* 	return VFS_EUNKNOWN; */

  if((node_pv = mem_alloc(sizeof(*node_pv), MEM_SCOPE_SYS)) == NULL)
    return VFS_ENOMEM;

#ifdef CONFIG_DEVFS_DEBUG
  printf("init_devfs: node_pv allocated\n");
#endif

  memset(node_pv, 0, sizeof(*node_pv));
  node->n_pv = (void *) node_pv;
  return 0;
}

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_RELEASE_NODE(devfs_release_node)
{
  if(node->n_pv == NULL)
    return 0;

#ifdef CONFIG_DEVFS_DEBUG
  printf("+++++ devfs_release_node: freeing devfs_node_pv\n");
#endif
  mem_free(node->n_pv);
  node->n_pv = NULL;

  return 0;
}

/*
** param	struct vfs_node_s *parent
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_CREATE_NODE(devfs_create_node)
{
/*   struct devfs_node_s	*parent_pv = parent->n_pv; */

#ifdef CONFIG_DEVFS_DEBUG
  printf("+++++ devfs_create_node: creating devfs_node\n");
#endif

  return 0;
}

/*
** param	struct vfs_node_s *parent
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_LOOKUP_NODE(devfs_lookup_node)
{
/*   struct devfs_node_s	*parent_pv = parent->n_pv; */
/*   struct devfs_node_s	*node_pv = node->n_pv; */

  return 0;
}

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_WRITE_NODE(devfs_write_node)
{
    return 0;
}

/*
** param	struct vfs_node_s *node
** return	error_t
*/
VFS_UNLINK_NODE(devfs_unlink_node)
{
/*   struct vfs_node_s	*tmp_node; */

#ifdef CONFIG_DEVFS_DEBUG
  printf("+++++ devfs_create_node: creating devfs_node\n");
#endif
  
  // Get node from children list "n_children" in parent's node
/*   if ((tmp_node = vfs_node_list_get(&vfs_dev_node->n_children)) == NULL) */
/*     return VFS_EUNKNOW; */

  // Set back node in freelist
/*   vfs_node_list_push(&dev_node->n_freelist, tmp_node); */

  return 0;
}
