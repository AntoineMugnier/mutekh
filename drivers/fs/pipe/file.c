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
#include <mutek/mem_alloc.h>
#include <mutek/rwlock.h>
#include "pipe.h"


VFS_OPEN_FILE(pipe_open)
{
#if PIPE_DEBUG
  printk("pipe_open: opinign file in RD_ONLY ? %d\n", 
	 VFS_IS(file->f_flags,VFS_O_RDONLY));
#endif

  rwlock_wrlock(&file->f_node->n_lock);

#if PIPE_DEBUG
  printk("pipe_open: node %s locked\n",file->f_node->n_name);
#endif

  if(VFS_IS(file->f_flags, VFS_O_RDONLY)){
    file->f_node->n_readers ++;
#if PIPE_DEBUG
    printk("pipe_open: node %s readers is now %d\n",
	   file->f_node->n_name,file->f_node->n_readers);
#endif
  }
  if(VFS_IS(file->f_flags, VFS_O_WRONLY)){
    file->f_node->n_writers ++;
#if PIPE_DEBUG
    printk("pipe_open: node %s writers is now %d\n",
	   file->f_node->n_name,file->f_node->n_writers);
#endif
  }

  while((VFS_IS(file->f_flags, VFS_O_RDONLY)) && (!file->f_node->n_writers))
  {
#if PIPE_DEBUG
    printk("pipe_open: going to wait for at least one writer\n");
#endif
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    sched_wait_callback(&file->f_node->n_wait, (sched_wait_cb_t*)rwlock_unlock, &file->f_node->n_lock);
    CPU_INTERRUPT_RESTORESTATE;
    rwlock_wrlock(&file->f_node->n_lock);
  }

  while((VFS_IS(file->f_flags, VFS_O_WRONLY)) && (!file->f_node->n_readers))
  {
#if PIPE_DEBUG
    printk("pipe_open: going to wait for at least one reader\n");
#endif
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    sched_wait_callback(&file->f_node->n_wait, (sched_wait_cb_t*)rwlock_unlock, &file->f_node->n_lock);
    CPU_INTERRUPT_RESTORESTATE;
    rwlock_wrlock(&file->f_node->n_lock);
  }

#if PIPE_DEBUG
    printk("pipe_open: waking up all waitig \"process\"\n");
#endif

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  while(sched_wake(&file->f_node->n_wait));
  CPU_INTERRUPT_RESTORESTATE;

  rwlock_unlock(&file->f_node->n_lock);
  VFS_SET(file->f_flags, VFS_O_FIFO);
#if PIPE_DEBUG
  printk("pipe_open: node %s unlocked, f_flags %d\n",
	 file->f_node->n_name,file->f_flags);
#endif
  return 0;
}


VFS_READ_FILE(pipe_read)
{
  struct pipe_node_s *node_info;
  uint8_t *fifo;
  size_t pRead;
  size_t status;
  size_t bytes_left;
  size_t pipe_size;
  size_t to_read;
  size_t asked_size;

  if(file->f_node->n_writers == 0)
    return 0;

#if PIPE_DEBUG
  printk("pipe_read: there is at least on writer, trying to read %d byets\n",size);
#endif

  rwlock_unlock(&file->f_node->n_lock);
  rwlock_unlock(&file->f_rwlock);

  node_info = file->f_node->n_pv;
  fifo = node_info->buffer->content;
  pipe_size = node_info->size;
  asked_size = size;

#if PIPE_DEBUG
  printk("pipe_read: pipe info :\npipe_size %d\nstatus %d\npRead %d\npWrite %d\n",
	 pipe_size, node_info->status, node_info->pRead, node_info->pWrite);
#endif
  
  rwlock_wrlock(&node_info->lock);
  
  while(size)
  {
    while(!node_info->status)
    {
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      sched_wait_callback(&node_info->rd_wait, (sched_wait_cb_t*)rwlock_unlock, &node_info->lock);
      CPU_INTERRUPT_RESTORESTATE;
    
      rwlock_wrlock(&node_info->lock);
    }
  
    pRead = node_info->pRead;
    status = node_info->status;
    bytes_left = pipe_size - pRead;
    to_read = (size > status) ? status : size; 
    
    if(to_read > bytes_left)
    {
      memcpy(buffer, fifo + pRead, bytes_left);
      memcpy(buffer + bytes_left, fifo, to_read - bytes_left);
    }
    else
      memcpy(buffer, fifo + pRead, to_read);

    pRead = (pRead + to_read) % pipe_size;
    status -= to_read;
    node_info->pRead = pRead;
    node_info->status = status;
    size -= to_read;
    buffer += to_read;

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    while(sched_wake(&node_info->wr_wait));
    CPU_INTERRUPT_RESTORESTATE;
  }
  rwlock_unlock(&node_info->lock);

  rwlock_wrlock(&file->f_rwlock);
  rwlock_rdlock(&file->f_node->n_lock);
  return asked_size - size;
}

VFS_WRITE_FILE(pipe_write)
{
  struct pipe_node_s *node_info;
  uint8_t *fifo;
  size_t pWrite;
  size_t status;
  size_t bytes_left;
  size_t pipe_size;
  size_t available_size;
  size_t to_write;
  size_t asked_size;

  if(file->f_node->n_readers == 0)
    return -VFS_EPIPE;
 
  rwlock_unlock(&file->f_node->n_lock);
  rwlock_unlock(&file->f_rwlock);
  
  node_info = file->f_node->n_pv;
  fifo = node_info->buffer->content;
  pipe_size = node_info->size;
  asked_size = size;

  rwlock_wrlock(&node_info->lock);
  while(size)
  {
    while(node_info->status >= pipe_size)
    {
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      sched_wait_callback(&node_info->wr_wait, (sched_wait_cb_t*)rwlock_unlock, &node_info->lock);
      CPU_INTERRUPT_RESTORESTATE;
      
      rwlock_wrlock(&node_info->lock);
    }

    pWrite = node_info->pWrite;
    status = node_info->status;
    bytes_left = pipe_size - pWrite;
    available_size = pipe_size - status;
    to_write = (size > available_size) ? available_size : size;
    
    if(to_write > bytes_left)
    {
      memcpy(fifo + pWrite, buffer, bytes_left);
      memcpy(fifo, buffer + bytes_left, to_write - bytes_left);
    }
    else
      memcpy(fifo + pWrite, buffer, to_write);

    pWrite = (pWrite + to_write) % pipe_size;
    status += to_write;
    node_info->pWrite = pWrite;
    node_info->status = status;
    size -= to_write;
    buffer += to_write;

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    while(sched_wake(&node_info->rd_wait));
    CPU_INTERRUPT_RESTORESTATE;
  }
  rwlock_unlock(&node_info->lock);

  rwlock_wrlock(&file->f_rwlock);
  rwlock_rdlock(&file->f_node->n_lock);
  return asked_size - size;
}


VFS_LSEEK_FILE(pipe_lseek)
{
  return VFS_EISPIPE;
}

VFS_RELEASE_FILE(pipe_release)
{
  return 0;
}

VFS_READ_DIR(pipe_readdir)
{
  return VFS_EISPIPE;
}
