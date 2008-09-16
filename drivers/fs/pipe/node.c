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
#include <vfs/buffer_cache.h>
#include "pipe.h"


VFS_INIT_NODE(pipe_init_node)
{
  struct pipe_node_s *node_info;
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];

  if(node->n_pv != NULL)
    return VFS_EUNKNOWN;

  if((node_info = mem_alloc(sizeof(*node_info), MEM_SCOPE_SYS)) == NULL)
    return VFS_ENOMEM;

#if PIPE_DEBUG
  printf("init_pipe: node_info allocated\n");
#endif

  request.key1 = (key_t) node_info;
  request.key2 = 0;
  request.buffers = buffers;
  request.count = 1;
  if(bc_get_buffer(&bc,&freelist,&request) == NULL){
    mem_free(node_info);
    return VFS_IO_ERR;
  }

#if PIPE_DEBUG
  printf("init_pipe: bc_buffer has been allocated\n");
#endif

  memset(node_info, 0, sizeof(*node_info));
  rwlock_init(&node_info->lock);
  sched_queue_init(&node_info->wr_wait);
  sched_queue_init(&node_info->rd_wait);

  node_info->size = bc.buffer_size;
  node_info->buffer = buffers[0];
  node->n_pv = (void *) node_info;
  return 0;
}


VFS_RELEASE_NODE(pipe_release_node)
{
  struct pipe_node_s *node_info;
  struct bc_request_s request;
  struct bc_buffer_s *buffers[1];
  
  node_info = node->n_pv;
  
  if(node_info == NULL)
    return 0;
#if PIPE_DEBUG
  printf("++++++ release_pipe started ++++ \n");
#endif
  rwlock_destroy(&node_info->lock);
  sched_queue_destroy(&node_info->wr_wait);
  sched_queue_destroy(&node_info->rd_wait);
  
  request.key1 = (key_t) node_info;
  request.key2 = 0;
  buffers[0] = node_info->buffer;
  request.buffers = buffers;
  request.count = 1;
  bc_release_buffer(&bc,&freelist,&request,1);
  
  mem_free(node_info);
  
  node->n_pv = NULL;
  return 0;
}


VFS_CREATE_NODE(pipe_create_node)
{ 
  return VFS_EISPIPE;
}


VFS_LOOKUP_NODE(pipe_lookup_node)
{
  return VFS_EISPIPE;
}


VFS_WRITE_NODE(pipe_write_node)
{
  return VFS_EISPIPE;
}

VFS_UNLINK_NODE(pipe_unlink_node)
{
  return VFS_EISPIPE;
}
