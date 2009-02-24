
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
    //if (mode & O_TRUNC) //FIXME: seems not to exist yet in libVFS
    //    flags |= VFS_O_TRUNC;
    if (mode & O_APPEND)
        flags |= VFS_O_APPEND;

    return (flags);
}

/* Process a path to be compliant with the 
 * stupid upper short names of VFAT.
 * 
 * This is a temporarily function waiting for
 * libVFS to support long filenames.
 *
 * 8 upper characters max before the extension:
 *  - blabla -> BLABLA
 *  - blablabla -> BLABLAB~1
 * 3 upper characters max for the extension:
 *  - blablabla.b -> BLABLAB~1.B
 *  - blabla.blabla -> BLABLA.BLA
 *
 *  ! This function processes directly the 
 *  string argument.
 */
void touppershortname(char* path)
{
    /* sanity check */
    assert(path != NULL);
    if (*path == 0) return;

    char *p = path;
    char *delim;
    delim = strchr(path, '.');

    size_t i;

    /* process the body name */
    for (i = 0; i < 8 && *p && (p != delim); i++, p++)
    {
        if ((delim - path) > 8 && i >= 6)
        {
            if (i == 6)
                *p = '~';
            else
                *p = '1';
        }
        else
            *p = toupper(*p);
    }

    /* process the extension if there is one */
    if (delim)
    {
        for (i = 0; i < 3 && *delim; i++, delim++, p++)
            *p = toupper(*delim);
    }

    /* end of string */
    *p = 0;
}

static fd_t vfs_fopen(const char *name, uint_fast8_t mode)
{
    struct vfs_file_s *file;

    if (vfs_open(vfs_root, name, mode_to_vfs(mode), 0, &file) == 0)
        return (fd_t)file;
    else
        return NULL;
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
    if (vfs_lseek((struct vfs_file_s*)fd, offset, whence) == 0)
        return ((struct vfs_file_s*)fd)->f_offset;
    else
        return -1;
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

