
#include <stdio.h>

#include <hexo/types.h>

#include <device/char.h>
#include <hexo/device.h>
#include <device/driver.h>

#include <vfs/vfs.h>

static uint_fast32_t mode_to_vfs(const uint_fast8_t mode)
{
    uint_fast32_t	flags = 0;

    if (mode & O_RDONLY)
        flags |= VFS_O_RDONLY;
    if (mode & O_WRONLY)
        flags |= VFS_O_WRONLY;
    if (mode & O_CREAT)
        flags |= VFS_O_CREATE;
    //if (mode & O_TRUNC)
    //    flags |= VFS_O_TRUNC;
    if (mode & O_APPEND)
        flags |= VFS_O_APPEND;

    return (flags);
}

static fd_t vfs_fopen(const char *name, uint_fast8_t mode)
{
    struct vfs_file_s *file;
    error_t error;

    error = vfs_open(vfs_root, name, mode_to_vfs(mode), 0, &file);
    if (error)
        return NULL;
    else
        return (fd_t)file;
}

static ssize_t vfs_fwrite(fd_t fd, const void *buffer, size_t count)
{
    return vfs_write((struct vfs_file_s*)fd, buffer, count);
    
}

static ssize_t vfs_fread(fd_t fd, void *buffer, size_t count)
{
    return vfs_read((struct vfs_file_s*)fd, buffer, count);
    
}

static error_t vfs_fclose(fd_t fd)
{
    return vfs_close((struct vfs_file_s*)fd);
    
}

static off_t vfs_fseek(fd_t fd, off_t offset, enum stream_whence_e whence)
{
    return vfs_lseek((struct vfs_file_s*)fd, offset, whence);
}

static bool_t	true_able(fd_t fd)
{
  return 1;
}

const struct stream_ops_s vfs_ops =
{
    .open       = &vfs_fopen,
    .write      = &vfs_fwrite,
    .read       = &vfs_fread,
    .close      = &vfs_fclose,
    .lseek      = &vfs_fseek,
    .readable   = &true_able,
    .writable   = &true_able,
};
