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
    Copyright Sylvain Leroy <sylvain.leroy@unmondelibre.fr>
*/

#include <hexo/alloc.h>
#include "devfs.h"

VFS_OPEN_FILE(devfs_open)
{
    return 0;
}


VFS_READ_FILE(devfs_read)
{
    size_t	s = 0;

#if DEVFS_DEBUG
    printf("++++ devfs_read started, asked size %d\n", size);
#endif

    if (file->f_node->n_attr & VFS_FIFO)
	return -EINVAL;

    if (size == 0)
	return 0;

    if (node->n_pv->type == DEVFS_CHAR)
    {
#if DEVFS_DEBUG
	printf("++++ devfs_read on type char\n");
#endif
	if ((s = dev_char_read(*(struct device) file, buffer, size)) < 0)
	    return EIO;
    }
    return s;
}

VFS_WRITE_FILE(devfs_write)
{
    size_t	s = 0;

#if DEVFS_DEBUG
    printf("++++ devfs_write started, asked size %d\n", size);
#endif

    if (node->n_pv->type == DEVFS_CHAR)
    {
#if DEVFS_DEBUG
	printf("++++ devfs_write on type char\n");
#endif
	if ((s = dev_char_write(*(struct device) file, buffer, size)) < 0)
	    return EIO;
    }
    return s;
}

VFS_LSEEK_FILE(devfs_lseek)
{
  return 0;
}

VFS_RELEASE_FILE(devfs_release)
{
  return 0;
}

VFS_READ_DIR(devfs_readdir)
{
  return 0;
}
