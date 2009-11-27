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

/**
   @file
   @module {Virtual File System}
   @short Keep track of global file system root and cwd nodes
 */

#ifndef _VFS_GLOBALS_H_
#define _VFS_GLOBALS_H_

#include <hexo/local.h>

struct vfs_node_s;

/** @this holds pointer to file system root and current working directory */
struct vfs_state_s
{
	struct vfs_node_s *root;
	struct vfs_node_s *cwd;
};

extern CONTEXT_LOCAL struct vfs_state_s vfs_state;

/** @this returns the global vfs root node */
static inline
struct vfs_node_s *vfs_get_root()
{
	struct vfs_state_s *state = &CONTEXT_LOCAL_GET(vfs_state);
	return state->root;
}

/** @this change the global vfs root node */
static inline
void vfs_set_root(struct vfs_node_s *root)
{
	struct vfs_state_s *state = &CONTEXT_LOCAL_GET(vfs_state);
	struct vfs_node_s *old = state->root;
	if ( root )
		state->root = vfs_node_refnew(root);
	else
		state->root = NULL;

	if ( old )
		vfs_node_refdrop( old );
}

/** @this returns the global current working directory node */
static inline
struct vfs_node_s *vfs_get_cwd()
{
	struct vfs_state_s *state = &CONTEXT_LOCAL_GET(vfs_state);
	return state->cwd;
}

/** @this change the global current working directory node */
static inline
void vfs_set_cwd(struct vfs_node_s *cwd)
{
	struct vfs_state_s *state = &CONTEXT_LOCAL_GET(vfs_state);
	struct vfs_node_s *old = state->cwd;
	if ( cwd )
		state->cwd = vfs_node_refnew(cwd);
	else
		state->cwd = NULL;

	if ( old )
		vfs_node_refdrop( old );
}

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

