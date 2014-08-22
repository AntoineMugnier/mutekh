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

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009,2014
*/

#include <vfs/name.h>

bool_t vfs_name_compare(const char *fullname, size_t fulllen,
                        const char *vfsname, size_t vfsnamelen)
{
    /* should compare vfsname with both fullname and shortened name */

    assert(vfsnamelen && fulllen);

    if (fulllen == vfsnamelen && !memcmp(fullname, vfsname, fulllen))
        return 1;

    if (fulllen > CONFIG_VFS_NAMELEN) {
        /* FIXME should gracefully handle shortened names compare HERE */
    }

    return 0;
}

size_t vfs_name_mangle(const char *fullname, size_t fulllen, char *vfsname)
{
    /* FIXME should gracefully handle shortened names colision */
    if (fulllen > CONFIG_VFS_NAMELEN)
        fulllen = CONFIG_VFS_NAMELEN;
    /* fulllen can be 0 for anonymous nodes only */
    memcpy(vfsname, fullname, fulllen);
	memset(vfsname + fulllen, 0, CONFIG_VFS_NAMELEN - fulllen);

    return fulllen;
}

