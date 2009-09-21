
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_darray.h>

#include <unistd.h>
#include <fileops.h>

#ifdef CONFIG_VFS
#include <vfs/vfs.h>
#endif

struct fd_entry_s
{
  struct fileops_s *ops;
  void *hndl;
};

CONTAINER_TYPE(fdarray, DARRAY, struct fd_entry_s, 1, 256)
CONTAINER_FUNC(fdarray, DARRAY, static, fdarray);

static fdarray_root_t fd_array;

#ifdef CONFIG_VFS

static fd_t fd_new(fdarray_root_t *fda)
{
  fdarray_index_t i;

  for (i = 0; i < fdarray_count(fda); i++)
    if (fdarray_getptr(fda, i)->hndl == NULL)
      return i;

  return fdarray_alloc(fda);
}

#endif

static struct fd_entry_s * fd_get(fdarray_root_t *fda, fd_t fd)
{
  if (fd > fdarray_count(fda))
    return NULL;

  return fdarray_getptr(fda, fd);
}

static void fd_free(fdarray_root_t *fda, struct fd_entry_s *e)
{
  e->hndl = NULL;
}

/* **********************************************************************
                  File descriptor oriented operations
   ********************************************************************** */

#ifdef CONFIG_VFS

static const struct fileops_s open_fops =
{
  .read = (fileops_read_t*)vfs_read,
  .write = (fileops_write_t*)vfs_write,
  .lseek = (fileops_lseek_t*)vfs_lseek,
  .close = (fileops_close_t*)vfs_close,
};

inline fd_t creat(const char *pathname, mode_t mode)
{
    return open(pathname, O_CREAT|O_WRONLY|O_TRUNC, mode);
}

static enum open_flags_e flags_to_vfs(const vfs_open_flags_t mode)
{
  vfs_open_flags_t flags = 0;

  if (mode & O_RDONLY)
    flags |= VFS_O_RDONLY;
  if (mode & O_WRONLY)
    flags |= VFS_O_WRONLY;
  if (mode & O_CREAT)
    flags |= VFS_O_CREATE;
  //if (mode & O_TRUNC) //FIXME: seems not to exist yet in libVFS
  //    flags |= VFS_O_TRUNC;
  if (mode & O_APPEND)
    flags |= VFS_O_APPEND;

  return (flags);
}

fd_t open(const char *pathname, enum open_flags_e flags, ...)
{
  fd_t fd = fd_new(&fd_array);
  struct fd_entry_s *e;
  mode_t mode = 0;
  va_list ap;

  if (fd < 0)
    return fd;

  e = fd_get(&fd_array, fd);
  e->ops = &open_fops;

  if (flags & O_CREAT)
    {
      va_start(ap, flags);
      mode = va_arg(ap, __compiler_sint_t);
      va_end(ap);
    }

  if (vfs_open(vfs_get_root(), pathname, flags_to_vfs(flags), mode, &e->hndl))
    {
      fd_free(&fd_array, e);
      return -1;
    }

  return fd;
}

#endif /* CONFIG_VFS */

off_t lseek(fd_t fd, off_t offset, enum seek_whence_e whence)
{
  struct fd_entry_s *e = fd_get(&fd_array, fd);

  return e && e->ops->lseek ? e->ops->lseek(e->hndl, offset, whence) : -1;
}

ssize_t read(fd_t fd, void *buf, size_t count)
{
  struct fd_entry_s *e = fd_get(&fd_array, fd);

  return e && e->ops->read ? e->ops->read(e->hndl, buf, count) : -1;
}

ssize_t write(fd_t fd, const void *buf, size_t count)
{
  struct fd_entry_s *e = fd_get(&fd_array, fd);

  return e && e->ops->write ? e->ops->write(e->hndl, buf, count) : -1;
}

error_t close(fd_t fd)
{
  struct fd_entry_s *e = fd_get(&fd_array, fd);

  if (e)
    {
      if (e->ops->close)
	{
	  e->ops->close(e->hndl);
	  fd_free(&fd_array, e);
	  return 0;
	}
    }
  return 1;
}

/* **********************************************************************
                  VFS operations
   ********************************************************************** */

#ifdef CONFIG_VFS

/* FIXME */
error_t stat(const char *path, struct stat *st)
{
  struct vfs_stat_s vst;

  if (vfs_stat(vfs_get_root(), path, &vst))
    return -1;

  memset(st, 0, sizeof(*st));
  st->st_size = vst.size;

  if (vst.attr & VFS_DIR)
    st->st_mode |= S_IFDIR;
  else
    st->st_mode |= S_IFREG;

  return 0;
}

error_t lstat(const char *path, struct stat *buf)
{
  return stat(path, buf);
}

error_t access(const char *pathname, enum access_perm_e mode)
{
  return 0;
}

error_t remove(const char *pathname)
{
  return vfs_unlink(vfs_get_root(), pathname);
}

error_t mkdir(const char *pathname, mode_t mode)
{
  return vfs_mkdir(vfs_get_root(), pathname, mode);
}

#endif /* CONFIG_VFS */

