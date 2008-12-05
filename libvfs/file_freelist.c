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
#include <hexo/lock.h>
#include <mutek/rwlock.h>
#include <vfs/vfs.h>
#include <vfs/vfs-private.h>


vfs_file_list_root_t vfs_file_list;
struct vfs_file_freelist_s vfs_file_freelist;

VFS_FILE_FREELIST_INIT(vfs_file_freelist_init)
{
  error_t err = 0;
  uint_fast8_t item;
  struct vfs_file_s *file;

  if((err=vfs_file_list_init(&vfs_file_list)))
    return err;

  if((err=vfs_file_list_init(&vfs_file_freelist.root)))
    return err;
  
  if((err=lock_init(&vfs_file_freelist.lock)))
    return err;

  for(item=0; item < length; item ++)
  {
    if((file=mem_alloc(sizeof(*file), MEM_SCOPE_SYS)) == NULL)
      return VFS_ENOMEM;
    
    memset(file, 0, sizeof(*file));

    if((err=rwlock_init(&file->f_rwlock)))
      return err;
    
    if((err=lock_init(&file->f_lock)))
      return err;

    vfs_file_list_push(&vfs_file_freelist.root, file);
  }
  
  return 0;
}

VFS_FILE_FREELIST_ADD(vfs_file_freelist_add)
{
  file->f_count = 0;
  file->f_offset = 0;
  file->f_flags = 0;
  file->f_mode = 0;
  file->f_version = 0;
  file->f_node = NULL;

  LOCK_SPIN_IRQ(&vfs_file_freelist.lock);  
  vfs_file_list_remove(&vfs_file_list,file);
  vfs_file_list_push(&vfs_file_freelist.root,file);
  LOCK_RELEASE_IRQ(&vfs_file_freelist.lock);
}

VFS_FILE_FREELIST_GET(vfs_file_freelist_get)
{
  struct vfs_file_s *file = NULL;

  lock_spin_irq(&vfs_file_freelist.lock);  
  
  if((file = vfs_file_list_pop(&vfs_file_freelist.root)) == NULL)
  {
    lock_release_irq(&vfs_file_freelist.lock);
    return NULL;
  }
#ifdef CONFIG_VFS_DEBUG
  printf("vfs_file_freelist_get: got a new file\n");
#endif
  //  print_file(file);
  vfs_file_list_push(&vfs_file_list,file);
  lock_release_irq(&vfs_file_freelist.lock);
  
  if(file->f_op != node->n_ctx->ctx_file_op)
  {
#ifdef CONFIG_VFS_DEBUG
  printf("vfs_file_freelist_get: file_op is not valid\n");
#endif
    if(file->f_op != NULL)
    {
      if(file->f_op->release(file))
	return NULL;
#ifdef CONFIG_VFS_DEBUG
  printf("vfs_file_freelist_get: release file private structure\n");
#endif
    }
#ifdef CONFIG_VFS_DEBUG
  printf("vfs_file_freelist_get: set file_op to node's context file_op\n");
#endif
    file->f_op = node->n_ctx->ctx_file_op;
  }
  
  file->f_count = 1;
  file->f_node = node;
  return file;
}

/* ================================================================ */


void vfs_print_file_freelist()
{
  printf("vfs_file_freelist: [");
  uint8_t i=0;
  CONTAINER_FOREACH_NOLOCK(vfs_file_list,CLIST,&vfs_file_freelist.root,
  {
    i++;

  });
  printf("\b\b] %d elements\n",i);
}
