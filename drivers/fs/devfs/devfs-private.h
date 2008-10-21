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
tty
null
zero
random
urandom
stdin
stdout
stderr

 */

enum devfs_e{
    DEVFS_CHAR,
    DEVFS_BLOCK,
    DEVFS_LINK,
    DEVFS_DIR
};

struct devfs_context_s
{
    char*		name;
};

struct devfs_node_s
{
    devfs_e		type; // character, block, link, directory, ...
    char*		name;
    uint_fast16_t	flags;
/*     sched_queue_root_t	wr_wait; */
/*     sched_queue_root_t	rd_wait; */
/*     struct rwlock_s	lock; */
/*     struct bc_buffer_s*	buffer; */
};

struct devfs_file_s
{
    char*		name;
};

#endif /* __DEVFS_PRIVATE_H__ */
