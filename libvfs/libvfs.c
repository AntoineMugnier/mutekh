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

#include <hexo/alloc.h>
#include <vfs/vfs.h>
#include <vfs/vfs-private.h>

#ifdef CONFIG_DRIVER_FS_PIPE
struct vfs_context_s *vfs_pipe_ctx;
#endif

uint_fast8_t vfs_dir_count(char *path)
{
  uint_fast8_t count = 0;
  char *path_ptr = path;

  while((path_ptr = strchr(path_ptr, '/')) != NULL)
  {
    path_ptr ++;
    count ++;
  }

  return (count == 0) ? 1 : count;
}


void vfs_split_path(char *path, char **dirs)
{
  uint_fast8_t i=0;
  char *path_ptr = path;
  
  dirs[0] = path_ptr;
  dirs[1] = NULL;
  if((path_ptr[0] == '/') && (path_ptr[1] == '\0'))
    return;
 
  path_ptr = (path_ptr[0] == '/') ? path_ptr + 1 : path_ptr;
  i++;
  dirs[0] = path_ptr;
  
  while((path_ptr = strchr(path_ptr, '/')) != NULL)
  {
    *path_ptr = 0;
    dirs[i++] = ++path_ptr;
  }

  dirs[i] = NULL;
}

/* VFS_OPEN(n)  error_t (n) (struct vfs_node_s *cwd,	\ */
/* 			  char *path,			\ */
/* 			  uint_fast32_t flags,		\ */
/* 			  uint_fast16_t mode,		\ */
/* 			  struct vfs_file_s **file) */

error_t vfs_open (struct vfs_node_s * cwd, char *path,
		  vfs_open_flags_t flags, vfs_mode_t mode,
		  struct vfs_file_s ** file)
{
  struct vfs_node_s *node;
  struct vfs_file_s *file_ptr;
  char str[VFS_MAX_NAME_LENGTH * VFS_MAX_PATH_DEPTH + 1];
  char *dirs_ptr[vfs_dir_count(path) + 1];
  error_t err;
  bool_t isAbsolutePath;

  err = 0;
  file_ptr = NULL;
  strcpy(str,path);
  vfs_split_path(str,dirs_ptr);

  isAbsolutePath = (path[0] == '/') ? 1 : 0 ;

  if((err = vfs_node_load(cwd,dirs_ptr, flags, isAbsolutePath, &node)))
    return err;
  
  if((file_ptr = vfs_file_freelist_get(node)) == NULL)
    goto VFS_OPEN_ERROR;
  
  file_ptr->f_flags = flags & 0xFFFF0000;
  file_ptr->f_mode = mode;

  if((err = file_ptr->f_op->open(node,file_ptr)))
    goto VFS_OPEN_ERROR;

  if(VFS_IS(flags,VFS_O_APPEND))
    if((err = vfs_lseek(file_ptr,0,VFS_SEEK_END)))
      goto VFS_OPEN_ERROR;

  *file = file_ptr;
  return 0;

 VFS_OPEN_ERROR:
#ifdef CONFIG_VFS_DEBUG
  printk("vfs_open : error while doing open, code %d\n",err);
#endif
  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_down(node);
  rwlock_unlock(&vfs_node_freelist.lock);
  
  if(file_ptr == NULL)  return -VFS_ENOMEM;
  vfs_file_freelist_add(file_ptr);
  return err;
}


error_t vfs_chdir (struct vfs_node_s * cwd, char *pathname, struct vfs_node_s ** new_cwd)
{
  struct vfs_node_s *node;
  uint_fast32_t flags;
  error_t err;
  char str[VFS_MAX_NAME_LENGTH * VFS_MAX_PATH_DEPTH + 1];
  char *dirs_ptr[vfs_dir_count(pathname) + 1];
  bool_t isAbsolutePath;
  
  err = 0;
  flags = 0;
  strcpy(str,pathname);
  vfs_split_path(str,dirs_ptr);
  
  flags = VFS_DIR;
  
  isAbsolutePath = (pathname[0] == '/') ? 1 : 0 ;
  if((err = vfs_node_load(cwd,dirs_ptr, flags, isAbsolutePath, &node)))
    return err;
  
  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_down(cwd);
  rwlock_unlock(&vfs_node_freelist.lock);
  
  *new_cwd = node;
  return 0;
}


error_t vfs_pipe (struct vfs_file_s * pipefd[2])
{
#ifdef CONFIG_DRIVER_FS_PIPE
  struct vfs_node_s *node;
  struct vfs_file_s *fd_in;
  struct vfs_file_s *fd_out;

  rwlock_wrlock(&vfs_node_freelist.lock);
  if((node = vfs_node_freelist_get(vfs_pipe_ctx)) == NULL)
  {
    rwlock_unlock(&vfs_node_freelist.lock);
    return -VFS_ENOMEM;
  }
  rwlock_unlock(&vfs_node_freelist.lock);

  VFS_SET(node->n_attr,VFS_PIPE);

#ifdef CONFIG_VFS_DEBUG
  printk("vfs_pipe: got a node, do n_op->init(node)\n");
#endif
  
#ifdef CONFIG_VFS_DEBUG
  printk("vfs_pipe: do n_op->init(node) OK\n");
#endif

  vfs_node_up(node);
  vfs_node_up(node);
  
  node->n_readers = 1;
  node->n_writers = 1;
  
  if((fd_in = vfs_file_freelist_get(node)) == NULL)
    goto VFS_PIPE_ERROR_FILE;
   
  if((fd_out = vfs_file_freelist_get(node)) == NULL)
  {
    vfs_file_freelist_add(fd_in);
    goto VFS_PIPE_ERROR_FILE;
  }

  VFS_SET(fd_in->f_flags, VFS_O_PIPE | VFS_O_WRONLY);
  VFS_SET(fd_out->f_flags,VFS_O_PIPE | VFS_O_RDONLY);
  pipefd[0] = fd_out;
  pipefd[1] = fd_in;
  return 0;
 
 VFS_PIPE_ERROR_FILE:
  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_freelist_add(node,1);
  rwlock_unlock(&vfs_node_freelist.lock);
  return -VFS_ENOMEM;

#else
  return -VFS_ENOTUSED;
#endif
}


error_t vfs_mkfifo (struct vfs_node_s * cwd, char *pathname, vfs_mode_t mode)
{
#ifdef CONFIG_DRIVER_FS_PIPE
  struct vfs_node_s *node;
  uint_fast32_t flags;
  error_t err;
  char str[VFS_MAX_NAME_LENGTH * VFS_MAX_PATH_DEPTH + 1];
  char *dirs_ptr[vfs_dir_count(pathname) + 1];
  bool_t isAbsolutePath;
 
  err = 0;
  flags = 0;
  VFS_SET(flags, VFS_O_CREATE | VFS_O_EXCL | VFS_FIFO);
  strcpy(str,pathname);
  vfs_split_path(str,dirs_ptr);
  isAbsolutePath = (pathname[0] == '/') ? 1 : 0 ;
  
  if((err = vfs_node_load(cwd,dirs_ptr, flags, isAbsolutePath, &node)))
    return err;

  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_down(node);
  rwlock_unlock(&vfs_node_freelist.lock);
  return 0;
#else
  return -VFS_ENOTUSED;
#endif
}


error_t vfs_unlink (struct vfs_node_s * cwd, char *pathname)
{
  struct vfs_node_s *node;
  struct vfs_node_s *parent;
  uint_fast32_t flags;
  error_t err;
  char str[VFS_MAX_NAME_LENGTH * VFS_MAX_PATH_DEPTH + 1];
  char *dirs_ptr[vfs_dir_count(pathname) + 1];
  bool_t isAbsolutePath;

  err = 0;
  flags = VFS_O_UNLINK;
  strcpy(str,pathname);
  vfs_split_path(str,dirs_ptr);

  isAbsolutePath = (pathname[0] == '/') ? 1 : 0 ;

#ifdef CONFIG_VFS_DEBUG
  printk("vfs_unlink started\n");
#endif
  if((err = vfs_node_load(cwd,dirs_ptr, flags, isAbsolutePath, &node)))
    return err;

  if(VFS_IS(node->n_attr,VFS_DIR))
    goto VFS_UNLINK_ERROR;
  
  parent = node->n_parent;
  rwlock_wrlock(&vfs_node_freelist.lock);
  node->n_links --;
  if(node->n_links == 0)
  {
    VFS_SET(parent->n_state,VFS_INLOAD);
    vfs_node_list_remove(&parent->n_children, node);
#ifdef CONFIG_VFS_DEBUG
    printk("vfs_unlink: parent's (%s) state is seted to INLOAD, node %s  detached from his fathor list\n",
	   parent->n_name,node->n_name);
#endif
    rwlock_unlock(&vfs_node_freelist.lock);

    if((err=parent->n_op->unlink(node)))
      goto VFS_UNLINK_ERROR;
#ifdef CONFIG_VFS_DEBUG
    printk("vfs_unlink: node %s has been removed from it's parent children list");
#endif
    rwlock_wrlock(&vfs_node_freelist.lock);
    VFS_CLEAR(parent->n_state,VFS_INLOAD);
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    while(sched_wake(&node->n_wait));
    CPU_INTERRUPT_RESTORESTATE;
  }
  
  vfs_node_down(node);
  rwlock_unlock(&vfs_node_freelist.lock);
  return 0;

 VFS_UNLINK_ERROR:
  rwlock_wrlock(&vfs_node_freelist.lock);
  if(!err)
  {
    err = VFS_EISDIR;
    vfs_node_list_remove(&node->n_parent->n_children, node);
  }
  else
  {
    VFS_CLEAR(node->n_parent->n_state, VFS_INLOAD);
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    while(sched_wake(&node->n_wait));
    CPU_INTERRUPT_RESTORESTATE;
  }
  vfs_node_down(node);
  rwlock_unlock(&vfs_node_freelist.lock);

  return -err;
}


error_t vfs_close (struct vfs_file_s * file)
{
  uint_fast8_t count;
  
  LOCK_SPIN_IRQ(&file->f_lock);
  count = file->f_count--;
  LOCK_RELEASE_IRQ(&file->f_lock);

  if(count > 1) return 0;

#ifdef CONFIG_DRIVER_FS_PIPE
  if(VFS_IS(file->f_flags, VFS_O_PIPE))
  {
    rwlock_wrlock(&file->f_node->n_lock);
  
    if(VFS_IS(file->f_flags, VFS_O_RDONLY))
      file->f_node->n_readers --;

    if(VFS_IS(file->f_flags, VFS_O_WRONLY))
      file->f_node->n_writers --;

    rwlock_unlock(&file->f_node->n_lock);
  }
#endif  
  rwlock_wrlock(&vfs_node_freelist.lock);
  vfs_node_down(file->f_node);
  rwlock_unlock(&vfs_node_freelist.lock);

  vfs_file_freelist_add(file);
  return 0;
}

error_t vfs_closedir (struct vfs_file_s * file)
{
  if(!(VFS_IS(file->f_flags, VFS_O_DIRECTORY)))
    return -VFS_EBADF;

  return vfs_close(file);
}

error_t vfs_mkdir (struct vfs_node_s * cwd, char *pathname, vfs_mode_t mode)
{
  struct vfs_file_s *file;
  uint_fast32_t flags;
  error_t err;
  
  flags = 0;
  VFS_SET(flags,VFS_O_DIRECTORY | VFS_O_CREATE | VFS_O_EXCL | VFS_DIR);

  if((err=vfs_open(cwd,pathname,flags,mode,&file)))
    return err;

  return vfs_close(file);
}

error_t vfs_opendir (struct vfs_node_s * cwd, char *path,
		     vfs_mode_t mode, struct vfs_file_s ** file)
{
  error_t err;
  uint_fast32_t flags;
  
  err = 0;
  flags = 0x00;
  VFS_SET(flags, VFS_DIR);

  if((err = vfs_open(cwd,path,flags,mode,file)))
    return err;
  
  VFS_SET((*file)->f_flags,VFS_O_DIRECTORY);
  return 0;
}

error_t vfs_readdir (struct vfs_file_s * file, struct vfs_dirent_s * dirent)
{
  error_t err;
  
  err = 0;
  
  if(!(VFS_IS(file->f_flags, VFS_O_DIRECTORY))){
#ifdef CONFIG_VFS_DEBUG
    printk("here!!\n");
#endif
    return -VFS_EBADF;
  }

  rwlock_wrlock(&file->f_rwlock);
  err = file->f_op->readdir(file,dirent);
  rwlock_unlock(&file->f_rwlock);
  return -err;
}

/* VFS_READ(n)  ssize_t (n) (struct vfs_file_s *file,	\ */
/* 			  uint8_t *buffer,		\ */
/* 			  size_t count) */
ssize_t vfs_read (struct vfs_file_s * file, uint8_t * buffer, size_t count)
{
  size_t available_size;
  size_t size_to_read;
  ssize_t size;

  if(VFS_IS(file->f_flags,VFS_O_DIRECTORY))
    return -VFS_EISDIR;

  if(!(VFS_IS(file->f_flags,VFS_O_RDONLY)))
    return -VFS_EBADF;

  rwlock_wrlock(&file->f_rwlock);
  rwlock_rdlock(&file->f_node->n_lock);

  available_size = file->f_node->n_size - file->f_offset;
  size_to_read = (count >= available_size) ?  available_size : count;
#ifdef CONFIG_DRIVER_FS_PIPE
  size_to_read = (VFS_IS(file->f_flags,VFS_O_PIPE)) ? count : size_to_read;
#endif
#ifdef CONFIG_DRIVER_FS_DEV
  size_to_read = (VFS_IS(file->f_flags, VFS_O_DEVICE)) ? count : size_to_read;
#endif

  if((size = file->f_op->read(file,buffer,size_to_read)) < 0)
    goto VFS_READ_ERROR;
  
  file->f_offset += size;
  
 VFS_READ_ERROR:
  rwlock_unlock(&file->f_node->n_lock);
  rwlock_unlock(&file->f_rwlock);
  return size;
}


ssize_t vfs_write (struct vfs_file_s * file, uint8_t * buffer, size_t count)
{
  ssize_t size;

  if(VFS_IS(file->f_flags,VFS_O_DIRECTORY))
    return -VFS_EINVAL;

  if(!(VFS_IS(file->f_flags,VFS_O_WRONLY)))
    return -VFS_EBADF;

  rwlock_wrlock(&file->f_rwlock);
  rwlock_wrlock(&file->f_node->n_lock);

  if((size = file->f_op->write(file,buffer,count)) < 0)
    goto VFS_WRITE_ERROR;

  file->f_offset += size;
  
  if(file->f_offset > file->f_node->n_size)
    file->f_node->n_size = file->f_offset;
  
  VFS_SET(file->f_node->n_state,VFS_DIRTY);

 VFS_WRITE_ERROR:
  rwlock_unlock(&file->f_node->n_lock);
  rwlock_unlock(&file->f_rwlock);
  return size;
}


ssize_t vfs_lseek (struct vfs_file_s * file, size_t offset, uint_fast32_t whence)
{
  size_t old_offset;
  uint64_t new_offset;
  error_t err = 0;
  
  if(VFS_IS(file->f_flags,VFS_O_DIRECTORY))
    return -VFS_EBADF;

  /* shortcut */
  if (whence == VFS_SEEK_CUR && offset == 0)
    return file->f_offset;

  rwlock_wrlock(&file->f_rwlock);
  rwlock_wrlock(&file->f_node->n_lock);

  old_offset = new_offset = file->f_offset;

  switch(whence)
  {
  case VFS_SEEK_SET:
    new_offset = offset;
    break;
    
  case VFS_SEEK_CUR:
    new_offset += offset;
    break;
      
  case VFS_SEEK_END:
    new_offset = file->f_node->n_size + offset;
    break;
    
  default:
    err = -VFS_EINVAL;
    goto VFS_LSEEK_ERROR;
  }
  
  if( new_offset >= __MAXOF_TYPE(size_t))
  {
    err = -VFS_EOVERFLOW;
    goto VFS_LSEEK_ERROR;
  }

  file->f_offset = (size_t) new_offset;
  
  if((err=file->f_op->lseek(file)))
  {
    err = -err;
    file->f_offset = old_offset;
    goto VFS_LSEEK_ERROR;
  }

  if(new_offset > file->f_node->n_size)
    file->f_node->n_size = (size_t)new_offset;
  
  VFS_SET(file->f_node->n_state,VFS_DIRTY);

  rwlock_unlock(&file->f_node->n_lock);
  rwlock_unlock(&file->f_rwlock);
  return new_offset;

 VFS_LSEEK_ERROR:
  assert(err < 0);
  rwlock_unlock(&file->f_node->n_lock);
  rwlock_unlock(&file->f_rwlock);
  return err;  
}

error_t vfs_stat (struct vfs_node_s *cwd, char *pathname, struct vfs_stat_s *stat)
{
  return -1;
}

