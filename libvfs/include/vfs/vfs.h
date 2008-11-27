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

#ifndef __VFS_H_
#define __VFS_H_

#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/scheduler.h>
#include <hexo/rwlock.h>
#include <hexo/atomic.h>
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#define VFS_MAX_NAME_LENGTH      13
#define VFS_MAX_PATH_DEPTH       12
#define VFS_MAX_NODE_NUMBER      40
#define VFS_MAX_FILE_NUMBER      80


#define VFS_DEBUG            10

#define VFS_FOUND            0
#define VFS_NOT_FOUND        1
#define VFS_ENOMEM           2
#define VFS_ENOSPC           3
#define VFS_IO_ERR           4
#define VFS_EUNKNOWN         5
#define VFS_EINVAL           6
#define VFS_EBADBLK          7
#define VFS_EEXIST           8
#define VFS_EISDIR           9
#define VFS_EBADF            10
#define VFS_EISPIPE          11
#define VFS_EOVERFLOW        12
#define VFS_ENOTDIR          13
#define VFS_EODIR            14
#define VFS_ENOTUSED         15
#define VFS_EPIPE            16
#define VFS_ENOTEMPTY        17

#define VFS_DIR              0x00000001
#define VFS_RD_ONLY          0x00000002
#define VFS_SYS              0x00000004
#define VFS_ARCHIVE          0x00000008
#define VFS_PIPE             0x00000010
#define VFS_FIFO             0x00000020
#define VFS_DEVICE	     0x00000040

#define VFS_O_PIPE           0x00010000
#define VFS_O_FIFO           0x00030000
#define VFS_O_DIRECTORY      0x00040000
#define VFS_O_APPEND         0x00080000
#define VFS_O_RDONLY         0x00100000
#define VFS_O_WRONLY         0x00200000
#define VFS_O_RDWR           0x00300000
#define VFS_O_CREATE         0x00400000
#define VFS_O_EXCL           0x00800000
#define VFS_O_UNLINK         0x01000000
#define VFS_O_MOUNTPOINT     0X02000000

#define VFS_VFAT_TYPE        1
#define VFS_PIPE_TYPE        2
#define VFS_DEVICE_TYPE      4

#define VFS_SEEK_SET         0
#define VFS_SEEK_CUR         1
#define VFS_SEEK_END         2

#define VFS_FREE             1
#define VFS_VAILD            2
#define VFS_INLOAD           4
#define VFS_DIRTY            8

#define VFS_SET(state,flag)    (state) |= (flag)
#define VFS_IS(state,flag)     (state) & (flag)
#define VFS_CLEAR(state,flag)  (state) &= ~(flag)

struct vfs_node_s;
struct vfs_node_op_s;
struct vfs_file_op_s;
struct vfs_context_op_s;

struct vfs_context_s
{
  uint_fast8_t  ctx_type;
  struct device_s *ctx_dev;
  struct vfs_context_op_s *ctx_op;
  struct vfs_node_op_s *ctx_node_op;
  struct vfs_file_op_s *ctx_file_op;
  void *ctx_pv;
};

#ifdef CONFIG_DRIVER_FS_PIPE
extern struct vfs_context_s *vfs_pipe_ctx;
#endif


CONTAINER_TYPE(vfs_freeList,CLIST, struct vfs_freelist_item_s
{
  struct vfs_node_s *node;
  vfs_freeList_entry_t list_entry;
}, list_entry);

CONTAINER_FUNC(vfs_freeList, CLIST, static inline, vfs_freeList,list_entry);


struct vfs_node_freelist_s 
{
  struct rwlock_s lock;
  vfs_freeList_root_t root;
};

extern struct vfs_node_freelist_s vfs_node_freelist;


CONTAINER_TYPE(vfs_node_list, CLIST, struct vfs_node_s
{
  char n_name[VFS_MAX_NAME_LENGTH];
  uint_fast16_t n_count;
  uint_fast16_t n_links;
  size_t n_size;
  uint_fast32_t n_flags;
  uint_fast8_t  n_type;
  uint_fast16_t n_attr;
  uint_fast16_t n_state;
  uint_fast16_t n_readers;
  uint_fast16_t n_writers;
  sched_queue_root_t n_wait;
  struct rwlock_s n_lock;
  struct vfs_node_op_s *n_op;
  struct vfs_context_s *n_ctx;
  struct vfs_node_s *n_parent;
  struct vfs_node_s *n_mounted_point;
  vfs_node_list_root_t n_children;
  struct vfs_freelist_item_s n_entry;
  vfs_node_list_entry_t n_list_entry;
  void *n_pv;
} , n_list_entry);

CONTAINER_FUNC(vfs_node_list, CLIST, static inline, vfs_node_list);

extern struct vfs_node_s *vfs_root;

CONTAINER_TYPE(vfs_file_list, CLIST, struct vfs_file_s
{
  vfs_file_list_entry_t f_list_entry;
  lock_t f_lock;
  uint_fast8_t f_count;
  uint32_t f_offset;
  uint_fast32_t f_flags;
  uint_fast16_t f_mode;
  uint_fast32_t f_version;
  struct rwlock_s f_rwlock;
  struct vfs_node_s *f_node;
  struct vfs_file_op_s *f_op;
  void *f_pv;
} , f_list_entry);

CONTAINER_FUNC(vfs_file_list, CLIST, static inline, vfs_file_list);

struct vfs_file_freelist_s
{
  lock_t lock;
  vfs_file_list_root_t root;
};

extern vfs_file_list_root_t vfs_file_list;
extern struct vfs_file_freelist_s vfs_file_freelist;

struct vfs_dirent_s
{
  uint_fast16_t d_type;
  size_t d_size;
  char d_name[VFS_MAX_NAME_LENGTH];
};

#define VFS_CREATE_CONTEXT(n)   error_t (n) (struct vfs_context_s *context)
#define VFS_DESTROY_CONTEXT(n)  error_t (n) (struct vfs_context_s *context)
#define VFS_READ_ROOT(n)        error_t (n) (struct vfs_context_s *context, \
					     struct vfs_node_s *root)
#define VFS_WRITE_ROOT(n)       error_t (n) (struct vfs_context_s *context, \
					     struct vfs_node_s *root)

typedef VFS_CREATE_CONTEXT(vfs_create_context_t);
typedef VFS_DESTROY_CONTEXT(vfs_destroy_context_t);
typedef VFS_READ_ROOT(vfs_read_root_t);
typedef VFS_WRITE_ROOT(vfs_write_root_t);

struct vfs_context_op_s
{
  vfs_create_context_t *create;
  vfs_destroy_context_t *destroy;
  vfs_read_root_t *read_root;
  vfs_write_root_t *write_root;
};

#define VFS_INIT_NODE(n)     error_t (n) (struct vfs_node_s *node)

#define VFS_CREATE_NODE(n)   error_t (n) (struct vfs_node_s *parent,	\
					  struct vfs_node_s *node)
#define VFS_LOOKUP_NODE(n)   error_t (n) (struct vfs_node_s *parent,    \
					  struct vfs_node_s *node)
#define VFS_WRITE_NODE(n)    error_t (n) (struct vfs_node_s *node)

#define VFS_RELEASE_NODE(n)  error_t (n) (struct vfs_node_s *node)

#define VFS_UNLINK_NODE(n)   error_t (n) (struct vfs_node_s *node)

typedef VFS_INIT_NODE(vfs_init_node_t);
typedef VFS_CREATE_NODE(vfs_create_node_t);
typedef VFS_LOOKUP_NODE(vfs_lookup_node_t);
typedef VFS_WRITE_NODE(vfs_write_node_t);
typedef VFS_RELEASE_NODE(vfs_release_node_t);
typedef VFS_UNLINK_NODE(vfs_unlink_node_t);

struct vfs_node_op_s
{
  vfs_init_node_t *init;
  vfs_create_node_t *create;
  vfs_lookup_node_t *lookup;
  vfs_write_node_t *write;
  vfs_release_node_t *release;
  vfs_unlink_node_t *unlink;
};


#define VFS_OPEN_FILE(n)    error_t (n) (struct vfs_node_s *node,	\
					 struct vfs_file_s *file)
#define VFS_READ_FILE(n)    error_t (n) (struct vfs_file_s *file,	\
					 uint8_t *buffer,		\
					 size_t size)
#define VFS_WRITE_FILE(n)   error_t (n) (struct vfs_file_s *file,	\
					 uint8_t *buffer,		\
					 size_t size)
#define VFS_LSEEK_FILE(n)   error_t (n) (struct vfs_file_s *file)

#define VFS_RELEASE_FILE(n) error_t (n) (struct vfs_file_s *file)

#define VFS_READ_DIR(n)     error_t (n) (struct vfs_file_s *file,	\
					 struct vfs_dirent_s *dirent)

typedef VFS_OPEN_FILE(vfs_open_file_t);
typedef VFS_READ_FILE(vfs_read_file_t);
typedef VFS_WRITE_FILE(vfs_write_file_t);
typedef VFS_LSEEK_FILE(vfs_lseek_file_t);
typedef VFS_RELEASE_FILE(vfs_release_file_t);
typedef VFS_READ_DIR(vfs_read_dir_t);

struct vfs_file_op_s
{
  vfs_open_file_t *open;
  vfs_read_file_t *read;
  vfs_write_file_t *write;
  vfs_lseek_file_t *lseek;
  vfs_read_dir_t *readdir;
  vfs_release_file_t *release;
};

#define VFS_INIT(n)  error_t (n) (struct device_s *device,	\
				  uint_fast8_t fs_type,         \
				  uint_fast8_t node_nr,		\
				  uint_fast16_t file_nr,	\
				  struct vfs_node_s **root)

#define VFS_CREATE(n) error_t (n)(struct vfs_node_s *cwd,	\
				  char *path,			\
				  uint_fast32_t flags,		\
				  uint_fast16_t mode,		\
				  struct vfs_file_s **file) 


#define VFS_OPEN(n)  error_t (n) (struct vfs_node_s *cwd,	\
				  char *path,			\
				  uint_fast32_t flags,		\
				  uint_fast16_t mode,		\
				  struct vfs_file_s **file)

#define VFS_READ(n)  ssize_t (n) (struct vfs_file_s *file,	\
				  uint8_t *buffer,		\
				  size_t count)

#define VFS_WRITE(n) ssize_t (n) (struct vfs_file_s *file,	\
				  uint8_t *buffer,		\
				  size_t count)

#define VFS_LSEEK(n) error_t (n) (struct vfs_file_s *file,	\
				  size_t offset,		\
				  uint_fast32_t whence)

#define VFS_CLOSE(n) error_t (n) (struct vfs_file_s *file)

#define VFS_UNLINK(n) error_t (n) (struct vfs_node_s *cwd,	\
				   char *pathname)

#define VFS_OPENDIR(n) error_t (n) (struct vfs_node_s *cwd,	\
				    char *path,			\
				    uint_fast16_t mode,		\
				    struct vfs_file_s **file)

#define VFS_READDIR(n) error_t (n) (struct vfs_file_s *file,	\
				    struct vfs_dirent_s *dirent)

#define VFS_MKDIR(n)   error_t (n) (struct vfs_node_s *cwd,	\
				    char *pathname,		\
				    uint_fast16_t mode)

#define VFS_CHDIR(n) error_t (n) (char *pathname,		\
				  struct vfs_node_s *cwd,	\
				  struct vfs_node_s **new_cwd)

#define VFS_CLOSE_DIR(n) error_t (n) (struct vfs_file_s *file)

#define VFS_PIPE_OPEN(n)   error_t (n) (struct vfs_file_s *pipefd[2])

#define VFS_MKFIFO(n)      error_t (n) (struct vfs_node_s *cwd,		\
					char *pathname,			\
					uint_fast16_t mode)


VFS_INIT(vfs_init);
VFS_OPEN(vfs_open);
VFS_READ(vfs_read);
VFS_WRITE(vfs_write);
VFS_LSEEK(vfs_lseek);
VFS_CLOSE(vfs_close);
VFS_UNLINK(vfs_unlink);

VFS_OPENDIR(vfs_opendir);
VFS_READDIR(vfs_readdir);
VFS_MKDIR(vfs_mkdir);
VFS_CHDIR(vfs_chdir);
VFS_CLOSE_DIR(vfs_closedir);

VFS_PIPE_OPEN(vfs_pipe);
VFS_MKFIFO(vfs_mkfifo);


// Used for devFS to always have root_node pointer
struct vfs_node_s	*vfs_get_root();

#endif
