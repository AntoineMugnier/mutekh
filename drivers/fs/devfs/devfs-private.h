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
#include <gpct/cont_hashlist.h>
#include <vfs/vfs-private.h>

// Return code
#define DEVFS_OK		0
#define DEVFS_ERR		1
#define DEVFS_DIREMPTY		14 // to match VFS_EODIR


// Values for  devfs_node_s->type;
#define DEVFS_DIR		0x00
#define DEVFS_CHAR		0x01
#define DEVFS_BLOCK		0x02


// temporary stuff
#define DEVFS_MOUNT_POINT	"/DEV"


struct devfs_node_s
{
  CONTAINER_ENTRY_TYPE(HASHLIST)        hash_entry;
  struct device_s			*device;
  const char				*name;
  uint_fast8_t				type;
};

CONTAINER_TYPE    (devfs_hash, HASHLIST, struct devfs_node_s, hash_entry, 111);
CONTAINER_KEY_TYPE(devfs_hash, STRING, name);

CONTAINER_FUNC    (devfs_hash, HASHLIST, static inline, devfs_hashfunc, name);
CONTAINER_KEY_FUNC(devfs_hash, HASHLIST, static inline, devfs_hashfunc, name);

struct devfs_context_s
{
  devfs_hash_root_t	hash;
};

struct devfs_file_s
{
  struct vfs_node_s	*node;
  struct devfs_node_s	*current_node; // used for readdir()
};

////////////////////////////////////////////////////

// Used to get DevFS context from anywhere
static inline struct devfs_context_s	*devfs_get_ctx()
{
  struct vfs_node_s		*dev_node = NULL;

  //get node to acces n_ctx field
  if ((dev_node = vfs_node_lookup(vfs_get_root(), "DEV")) == NULL)
    return NULL;

  return (dev_node->n_ctx->ctx_pv);
}

////////////////////////////////////////////////////

// Used to get DevFS node from anywhere
static inline struct vfs_node_s	*devfs_get_node()
{
  struct vfs_node_s		*dev_node = NULL;
  char				*t = NULL;

  if ((t = strrchr(DEVFS_MOUNT_POINT, '/')))
    {  
      if ((dev_node = vfs_node_lookup(vfs_get_root(), ++t)) == NULL)
	return NULL;
      else
	return (dev_node);
    }
  else
    {
      //get node to acces n_ctx field
      if ((dev_node = vfs_node_lookup(vfs_get_root(), "DEV")) == NULL)
	return NULL;
      else
	return (dev_node);
    }
}

////////////////////////////////////////////////////

#endif /* __DEVFS_PRIVATE_H__ */
