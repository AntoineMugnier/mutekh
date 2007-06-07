#ifndef __VFS_H_
#define __VFS_H_

#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/fs.h>
#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>

#include <gpct/cont_clist.h>

struct vfs_handle_s
{
  struct fs_handle_s	handle;
  //  struct vfs_node_s *node;
  /* ... */
};

struct vfs_drv_s
{
  error_t	(*get_entity_info)(struct fs_disk_context_s *disk_context,
				   struct fs_entity_s *parent,
				   struct fs_entity_s *file,
				   char *fname);

  error_t	(*get_root_info)(struct fs_disk_context_s *disk_context,
				 struct fs_entity_s *file);

  struct fs_disk_context_s *(*context_create)(struct device_s *device);

  error_t	(*context_destroy)(struct fs_disk_context_s *disk_context);
  /* ... */
};

struct vfs_context_s
{
  struct fs_disk_context_s	*dsk;
  const struct vfs_drv_s	*drv;
  struct vfs_node_s		*mpoint;
  struct vfs_node_s		*tmp_chld;
  struct vfs_context_s		*tmp_ctx;
};

CONTAINER_TYPE(vfs_node_list, CLIST, struct vfs_node_s
{
  struct fs_entity_s		ent;		/* the filesystem entity itself */
  vfs_node_list_entry_t		list_entry;
  struct vfs_context_s		*ctx;		/* ptr to the file system instance */
  struct vfs_node_s		*parent;	/* ptr to parent node */
  vfs_node_list_root_t		children;	/* children list */
} , list_entry);

CONTAINER_FUNC(vfs_node_list, CLIST, static inline, vfs_node_func);

/* node.c */
struct vfs_node_s *vfs_get_node(struct vfs_node_s *parent,
				char *name);

/* libvfs.c */
struct vfs_node_s *vfs_load(struct vfs_node_s *root_node,
			    char *tpath[],
			    error_t *ec);

/* toolkit.c */
void vfs_debug_tree(struct vfs_node_s *root_node);
uint16_t vfs_tok_count(char *path, char token);
void vfs_tokenize_path(char *path, char *t[], char token);

/* libVFS_init.c */
error_t vfs_init(struct vfs_node_s *root,
		 const struct vfs_drv_s *drv,
		 struct device_s *device);

/* vfs_open.c */

/* vfs_stat.c */

#endif
