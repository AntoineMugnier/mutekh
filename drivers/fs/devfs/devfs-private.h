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

#ifndef __DEVFS_PRIVATE_H__
#define __DEVFS_PRIVATE_H__

#include <hexo/types.h>
#include <vfs/vfs.h>

/*

needed nodes :
null
zero
random
urandom
tty
- stdin
- stdout
- stderr

 */

enum devfs_type_e{
  devfs_char,
  devfs_block,
  devfs_link,
  devfs_dir
};

struct devfs_context_s
{
};

struct devfs_node_s
{
  enum devfs_type_e	type;
  struct device_s	*device;
};

struct devfs_file_s
{
};

#endif /* __DEVFS_PRIVATE_H__ */
