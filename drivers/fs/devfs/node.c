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
//#include <vfs/buffer_cache.h>
#include "devfs.h"


VFS_INIT_NODE(devfs_init_node)
{
    struct devfs_node_s *node_info;

/*     if(node->n_pv != NULL) */
/* 	return VFS_EUNKNOWN; */

    if((node_info = mem_alloc(sizeof(*node_info), MEM_SCOPE_SYS)) == NULL)
	return VFS_ENOMEM;

#if DEVFS_DEBUG
    printf("init_devfs: node_info allocated\n");
#endif

    memset(node_info, 0, sizeof(*node_info));
    node->n_pv = (void *) node_info;
    return 0;
}


VFS_RELEASE_NODE(devfs_release_node)
{
    if(node->n_pv == NULL)
	return 0;

#if VFAT_DEBUG
    printf("+++++ devfs_release_node: freeing devfs_node_info\n");
#endif

    mem_free(node->n_pv);
    node->n_pv = NULL;

    return 0;
}


VFS_CREATE_NODE(devfs_create_node)
{
    return 0;
}


VFS_LOOKUP_NODE(devfs_lookup_node)
{
    return 0;
}


VFS_WRITE_NODE(devfs_write_node)
{
    return 0;
}

VFS_UNLINK_NODE(devfs_unlink_node)
{
    return 0;
}
