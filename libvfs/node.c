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


VFS_NODE_INIT(vfs_node_init)
{
  error_t err = 0;
  
  memset(node,0,sizeof(*node));
  
  if((err=sched_queue_init(&node->n_wait)))
    return err;

  if((err=rwlock_init(&node->n_lock)))
    return err;
  
  node->n_count = 1;
  node->n_op = ctx->ctx_node_op;
  node->n_ctx = ctx;
  node->n_type = ctx->ctx_type;

  if((err=vfs_node_list_init(&node->n_children)))
    return err;
  
  if((err=sched_queue_init(&node->n_wait)))
    return err;
  
  node->n_entry.node = node;  
  return (node->n_op->init(node));
}


VFS_NODE_LOOKUP(vfs_node_lookup)
{
  bool_t isDotDot;

  if(((name[0] == '/') || (name[0] == '.')) && (name[1] == '\0'))
    return node;

  isDotDot = ((name[0] == '.') && (name[1] == '.')) ? 1 : 0;

  if(isDotDot && (node == vfs_root))
    return node;
  
  if(isDotDot)
    return node->n_parent;

  CONTAINER_FOREACH_NOLOCK(vfs_node_list,CLIST,&node->n_children,
  {
    if(!strcmp(item->n_name,name))
      return item;
  });
  return NULL;
}


VFS_NODE_UP(vfs_node_up)
{
  node->n_count ++;

#ifdef CONFIG_VFS_DEBUG
  printf("+++++++ UP NODE %s, %d +++++\n",node->n_name,node->n_count);
#endif
}

VFS_NODE_DOWN(vfs_node_down)
{
  while(node!= NULL)
  {
#ifdef CONFIG_VFS_DEBUG
    printf("+++++++ DOWN NODE %s, %d ++++++\n",node->n_name,node->n_count -1);
#endif

    if((--node->n_count)) break;

    if(node->n_links == 0)
    {
      rwlock_unlock(&vfs_node_freelist.lock);
      node->n_parent->n_op->unlink(node);
      rwlock_wrlock(&vfs_node_freelist.lock);
      vfs_node_freelist_add(node,1);
    }
    else
      vfs_node_freelist_add(node,0);

    node = node->n_parent;
  }
}


VFS_NODE_CREATE(vfs_node_create)
{
  error_t err = 0;
  
  node->n_attr = (isLast) ? flags & 0x0000FFFF : VFS_DIR;

  // Look if node already exists
  err=node->n_op->lookup(parent,node);

  // Other error than found/not found
  if((err != VFS_NOT_FOUND) && (err != VFS_FOUND))
    return -err;

  // Node found
  if((err == VFS_FOUND) && (flags & VFS_O_EXCL) && (flags & VFS_O_CREATE) && (isLast))
    return -VFS_EEXIST;

#ifdef CONFIG_VFS_DEBUG
  printf("node %s, found ? %d, isLast ? %d, VFS_O_CREATE ? %d, VFS_FIFO? %d\n",
	 node->n_name,err, isLast,flags & VFS_O_CREATE, node->n_attr & VFS_FIFO);
#endif

  // Node not found
  if((err == VFS_NOT_FOUND) && (flags & VFS_O_CREATE) && (isLast))
  {
    if((err=node->n_op->create(parent,node)))
      return -err;
    node->n_links = 1;
  }

#ifdef CONFIG_DRIVER_FS_PIPE
  if(VFS_IS(node->n_attr,VFS_FIFO) && (!(VFS_IS(flags,VFS_O_UNLINK))))
  {
#ifdef CONFIG_VFS_DEBUG
    printf("node %s, is a fifo. releasing it",node->n_name);
#endif
    if((err=node->n_op->write(node)))
      return -err;
    
    if((err=node->n_op->release(node)))
      return -err;

    node->n_type = vfs_pipe_ctx->ctx_type;
    node->n_ctx = vfs_pipe_ctx;
    node->n_op = vfs_pipe_ctx->ctx_node_op;
    err = - node->n_op->init(node);
  }
#endif
  return err;
}


VFS_NODE_LOAD(vfs_node_load)
{
  struct vfs_node_s *child;
  struct vfs_node_s *current_parent;
  uint_fast8_t i;
  bool_t isLast;
  error_t err = 0;
  
  if((isAbsolutePath))
  {
#ifdef CONFIG_VFS_DEBUG
    printf("path is absolute, cwd != root\n");
#endif
    current_parent = vfs_root;
  }
  else
    current_parent = root;

#ifdef CONFIG_VFS_DEBUG
  printf("current_parent %s\n",current_parent->n_name);
#endif
  
  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_up(current_parent);

  for(i=0; path[i] != NULL; i++)
  { 
    isLast = (path[i+1] == NULL) ? 1 : 0;
    if((child = vfs_node_lookup(current_parent,path[i])) == NULL)
    {
      if((child = vfs_node_freelist_get(current_parent->n_ctx)) == NULL)
      {
	err = -VFS_ENOMEM;
	goto VFS_NODE_LOAD_ERROR;
      }
      strcpy(child->n_name, path[i]);
      VFS_SET(child->n_state,VFS_INLOAD);
      vfs_node_list_push(&current_parent->n_children, child);
      rwlock_unlock(&vfs_node_freelist.lock);
#ifdef CONFIG_VFS_DEBUG
      printf("going to physical load of %s\n",child->n_name);
#endif
      err=vfs_node_create(current_parent, flags, isLast,child);
      
      rwlock_wrlock(&vfs_node_freelist.lock);
      VFS_CLEAR(child->n_state,VFS_INLOAD);
      if(err)
      {
	vfs_node_list_remove(&current_parent->n_children, child);
	vfs_node_freelist_add(child,1);
	CPU_INTERRUPT_SAVESTATE_DISABLE;
	while(sched_wake(&child->n_wait));
	CPU_INTERRUPT_RESTORESTATE;
	goto VFS_NODE_LOAD_ERROR;
      }
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      while(sched_wake(&child->n_wait));
      CPU_INTERRUPT_RESTORESTATE;
      child->n_parent = current_parent;
      goto VFS_NODE_LOAD_CONTINUE;
    }

    if(VFS_IS(child->n_state,VFS_INLOAD))
    {
#ifdef CONFIG_VFS_DEBUG
      printf("node %s is inload\n",child->n_name);
#endif
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      sched_wait_callback(&child->n_wait, (sched_wait_cb_t*)rwlock_unlock, &vfs_node_freelist.lock);
      CPU_INTERRUPT_RESTORESTATE;
      rwlock_wrlock(&vfs_node_freelist.lock);

      if((child=vfs_node_lookup(current_parent,path[i])) == NULL)
      {
	err = -VFS_IO_ERR;
	goto VFS_NODE_LOAD_ERROR;
      }
    }
    

    if(isLast && ((flags & VFS_DIR) != (child->n_attr & VFS_DIR)))
    {
      err = -VFS_EISDIR;
      if(VFS_IS(child->n_state,VFS_FREE))
	vfs_node_freelist_add(child,1);
      goto VFS_NODE_LOAD_ERROR;
    }
    

    if((flags & VFS_O_EXCL) && (flags & VFS_O_CREATE) && isLast)
    {
      err = -VFS_EEXIST;
      if(VFS_IS(child->n_state,VFS_FREE))
	vfs_node_freelist_add(child,1);
      goto VFS_NODE_LOAD_ERROR;
    }

    if(VFS_IS(child->n_state,VFS_FREE))
    {
#ifdef CONFIG_VFS_DEBUG
      printf("child %s is in the node_freelist\n",child->n_name);
#endif
      vfs_node_freelist_unlink(child);
    }
    else
    {
#ifdef CONFIG_VFS_DEBUG
      printf("node was in use\n");
#endif
      vfs_node_down(current_parent);
    }
    
  VFS_NODE_LOAD_CONTINUE:
    vfs_node_up(child);   
    rwlock_unlock(&vfs_node_freelist.lock);
    current_parent = child;
    rwlock_wrlock(&vfs_node_freelist.lock);
  }
  
  rwlock_unlock(&vfs_node_freelist.lock);
  *node = child;
  return 0;
  
 VFS_NODE_LOAD_ERROR:
  vfs_node_down(current_parent);
  rwlock_unlock(&vfs_node_freelist.lock);
#ifdef CONFIG_VFS_DEBUG
  printf("vfs_load: error while creating/loading in-core node %s\n",path[i]);
#endif
  return err;
}
