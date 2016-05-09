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

/**
   @file
   @module {Libraries::Virtual File System}
   @short Constant definitions
 */

#ifndef _VFS_DEFS_H_
#define _VFS_DEFS_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <hexo/types.h>
#include <hexo/error.h>

struct vfs_node_s;
struct vfs_fs_s;
enum vfs_node_type_e;
enum vfs_open_flags_e;
struct vfs_node_s;
struct vfs_file_s;
struct vfs_stat_s;
struct vfs_fs_ops_s;

typedef size_t vfs_file_size_t;

#if defined(CONFIG_VFS_STATS)
# define VFS_STATS_INC(obj, field) atomic_inc(&(obj)->field)
#else
# define VFS_STATS_INC(x, y) do{}while(0)
#endif

#ifdef CONFIG_VFS_VERBOSE
# include <mutek/scheduler.h>
# include <mutek/printk.h>
# define vfs_printk(fmt, x...) do{printk("%p: "fmt"\n", sched_get_current(), ##x);}while(0)
#else
# define vfs_printk(...) do{}while(0)
#endif

C_HEADER_END

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

