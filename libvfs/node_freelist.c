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
    Copyright Ghassan Almaless <ghassan.almaless@gmail.com>
*/

#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include <vfs/vfs.h>
#include <vfs/vfs-private.h>

struct vfs_node_freelist_s vfs_node_freelist;

VFS_NODE_FREELIST_INIT(vfs_node_freelist_init)
{
  error_t err = 0;
  uint_fast8_t item;
  struct vfs_node_s *node;
  
  if((err=vfs_freeList_init(&vfs_node_freelist.root)))
    return err;

  if((err=rwlock_init(&vfs_node_freelist.lock)))
    return err;

  for(item=0; item < length; item ++)
  {
    if((node=mem_alloc(sizeof(*node), MEM_SCOPE_SYS)) == NULL)
      return VFS_ENOMEM;
    
    if((err=vfs_node_init(ctx,node)))
      return err;
    
    vfs_freeList_push(&vfs_node_freelist.root, &node->n_entry);
  }
  
  return 0;
}

VFS_NODE_FREELIST_ADD(vfs_node_freelist_add)
{
  if(VFS_IS(node->n_attr,VFS_PIPE))
  { 
    node->n_op->release(node);
    vfs_node_list_remove(&node->n_parent->n_children,node);
  }

  if(hasError)
    vfs_freeList_push(&vfs_node_freelist.root,&node->n_entry);
  else
  {
    if(VFS_IS(node->n_state,VFS_DIRTY))
    {
      node->n_op->write(node);
      VFS_CLEAR(node->n_state,VFS_DIRTY);
    }
    vfs_freeList_pushback(&vfs_node_freelist.root,&node->n_entry);
  }

  VFS_SET(node->n_state,VFS_FREE);
}

VFS_NODE_FREELIST_GET(vfs_node_freelist_get)
{
  struct vfs_node_s *node = NULL;
  struct vfs_freelist_item_s *item;
  
  if((item = vfs_freeList_pop(&vfs_node_freelist.root)) == NULL)
    return NULL;

  node = item->node;
  if (node->n_parent)
    vfs_node_list_remove(&node->n_parent->n_children,node);

  node->n_state = 0;
  node->n_attr = 0;
  node->n_size = 0;
  node->n_flags = 0;
  node->n_links = 0;
  node->n_count = 0;
  node->n_readers = 0;
  node->n_writers = 0;
  node->n_parent = NULL;
  
  if(node->n_ctx != parent_ctx)
  {
    if(node->n_op->release(node))
      goto VFS_NODE_FREELIST_GET_ERROR;
    
    node->n_type = parent_ctx->ctx_type;
    node->n_ctx = parent_ctx;
    node->n_op = parent_ctx->ctx_node_op;
    
    if(node->n_op->init(node))
      goto VFS_NODE_FREELIST_GET_ERROR;
  }
  
  return node;

 VFS_NODE_FREELIST_GET_ERROR:
  vfs_node_freelist_add(node,1);
  return NULL;
}


VFS_NODE_FREELIST_UNLINK(vfs_node_freelist_unlink)
{
  CONTAINER_FOREACH_NOLOCK(vfs_freeList,CLIST,&vfs_node_freelist.root,
  {
    if(item->node == node)
    {
      VFS_CLEAR(node->n_state,VFS_FREE);
      vfs_freeList_remove(&vfs_node_freelist.root,item);
      CONTAINER_FOREACH_BREAK;
    }
  }); 
}


/* ================================================================ */


void vfs_print_node_freelist()
{
  printf("vfs_freelist: [");
  uint8_t i=0;
  CONTAINER_FOREACH_NOLOCK(vfs_freeList,CLIST,&vfs_node_freelist.root,
  {
    i++;
    if(item->node != NULL)
      printf("%s, ",item->node->n_name);
  });
  printf("\b\b] %d elements\n",i);
}
