/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.
    
  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
    
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#ifndef _VFS_PRIVATE_H_
#define _VFS_PRIVATE_H_

static inline
void vfs_node_reparent(struct vfs_node_s *node, struct vfs_node_s *parent)
{
	if ( node->parent != NULL )
		vfs_node_refdrop(node->parent);

	if ( parent != NULL )
		node->parent = vfs_node_refnew(parent);
	else
		node->parent = NULL;
}

#endif
