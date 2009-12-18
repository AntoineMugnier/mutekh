/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.
    
  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
    
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#include <hexo/types.h>
#include <vfs/fs.h>
#include "fat-types.h"
#include "fat-private.h"

VFS_FS_CREATE(fat_create)
{
    return -ENOTSUP;
}

VFS_FS_LINK(fat_link)
{
    return -ENOTSUP;
}

VFS_FS_MOVE(fat_move)
{
    return -ENOTSUP;
}

VFS_FS_UNLINK(fat_unlink)
{
    return -ENOTSUP;
}

VFS_FS_STAT(fat_stat)
{
    stat->type = node->type;
    stat->size = node->type == VFS_NODE_DIR ? 0 : node->file_size;
    stat->nlink = 1;

    return 0;
}
