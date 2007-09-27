#ifndef __VFS_H_
#define __VFS_H_

#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/fs.h>
#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>

#include <gpct/cont_clist.h>
#include <gpct/object_refcount.h>

struct vfs_handle_s
{
  struct fs_handle_s	fs;
  /* ... */
};

struct vfs_drv_s
{
  /* entity */
  error_t (*get_root_info)(struct fs_disk_context_s *disk_context,
			   struct fs_entity_s *entity);

  error_t (*get_entity_info)(struct fs_disk_context_s *disk_context,
			     struct fs_entity_s *parent,
			     struct fs_entity_s *entity,
			     char *filename);

  error_t (*init_entity)(struct fs_entity_s *entity);
  error_t (*destroy_entity)(struct fs_entity_s *entity);

  /* directory */
  error_t (*find_first_file)(struct fs_disk_context_s *disk_context,
			     struct fs_handle_s *parent,
			     struct fs_entity_s *entity,
			     char *filename);

  error_t (*find_next_file)(struct fs_disk_context_s *disk_context,
			    struct fs_handle_s *parent,
			    struct fs_entity_s *entity,
			    char *filename);

  /* context */
  struct fs_disk_context_s *(*context_create)(struct device_s *device);
  error_t (*context_destroy)(struct fs_disk_context_s *disk_context);

  /* handle */
  error_t (*init_handle)(struct fs_disk_context_s *disk_context,
			 struct fs_handle_s *handle,
			 struct fs_entity_s *entity);

  void (*release_handle)(struct fs_handle_s *handle);

  /* IO */
  error_t (*read)(struct fs_disk_context_s *disk_context,
		  struct fs_handle_s *handle,
		  uint8_t *buffer,
		  uint32_t size);
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

OBJECT_TYPE(vfs_node_obj, REFCOUNT, struct vfs_node_s);
OBJECT_PROTOTYPE(vfs_node_obj, static inline, vfs_node_obj_func);


CONTAINER_TYPE(vfs_node_list, CLIST, struct vfs_node_s
{
  struct fs_entity_s		ent;		/* the filesystem entity itself */
  vfs_node_obj_entry_t		obj_entry;
  vfs_node_list_entry_t		list_entry;
  struct vfs_context_s		*ctx;		/* ptr to the file system instance */
  struct vfs_node_s		*parent;	/* ptr to parent node */
  vfs_node_list_root_t		children;	/* children list */
} , list_entry);

CONTAINER_FUNC(vfs_node_list, CLIST, static inline, vfs_node_func);


static OBJECT_CONSTRUCTOR(vfs_node_obj)
{
  obj->parent = va_arg(ap, struct vfs_node_s *);
  obj->ctx = va_arg(ap, struct vfs_context_s *);
  struct fs_entity_s *ent = va_arg(ap, struct fs_entity_s *);

  if (ent != NULL)
    memcpy(&obj->ent, ent, sizeof(*ent));

  vfs_node_func_init(&obj->children);

  return 0;
};

static OBJECT_DESTRUCTOR(vfs_node_obj)
{
  assert(obj->parent != NULL);

  vfs_node_func_remove(&obj->parent->children, obj);
  vfs_node_func_destroy(&obj->children);
};

OBJECT_FUNC(vfs_node_obj, REFCOUNT, static inline, vfs_node_obj_func, obj_entry);

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
void vfs_print_node(struct vfs_node_s *node);

/* libVFS_init.c */
error_t vfs_init(struct vfs_node_s *root,
		 const struct vfs_drv_s *drv,
		 struct device_s *device);

/* vfs_open.c */
struct vfs_handle_s *vfs_open(struct vfs_node_s *root_node,
			      char *path,
			      uint_fast16_t mode);

struct vfs_handle_s *vfs_opendir(struct vfs_node_s *root_node,
				 char *path,
				 uint_fast16_t mode);

error_t vfs_close(struct vfs_handle_s *handle);
error_t vfs_closedir(struct vfs_handle_s *handle);

error_t vfs_find_first_file(struct vfs_handle_s *direntry,
			    struct fs_entity_s *entity,
			    char *filename);

error_t vfs_find_next_file(struct vfs_handle_s *direntry,
			   struct fs_entity_s *entity,
			   char *filename);

/* vfs_read.c */
error_t vfs_read(struct vfs_handle_s *handle,
		 uint8_t *buffer,
		 uint32_t size);

#endif
