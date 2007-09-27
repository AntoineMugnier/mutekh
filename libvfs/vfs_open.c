#include <vfs/vfs.h>
#include <hexo/alloc.h>

static inline struct vfs_handle_s *vfs_get_handle(struct vfs_node_s *node)
{
  assert(node != NULL);

  struct vfs_handle_s *handle = mem_alloc(sizeof(*handle), MEM_SCOPE_SYS);

  if (handle != NULL)
    {
      handle->fs.ent = (struct fs_entity_s *)node;

      if (node->ctx->drv->init_handle(node->ctx->dsk, &handle->fs, &node->ent) == 0)
	{
	  // refnew on node
	  vfs_node_obj_func_refnew(node);

	  return handle;
	}
      mem_free(handle);
    }
  return NULL;
}

static inline void vfs_release_handle(struct vfs_handle_s *handle)
{
  assert(handle != NULL);

  struct vfs_node_s *node = (struct vfs_node_s *)(handle->fs.ent);

  node->ctx->drv->release_handle(&handle->fs);
  mem_free(handle);

  // refdrop on node
  vfs_node_obj_func_refdrop(node);
}

struct vfs_handle_s *vfs_open(struct vfs_node_s *root_node,
			      char *path,
			      uint_fast16_t mode)
{
  assert(root_node != NULL);

  struct vfs_node_s *node;
  error_t e = 0;
  char *tkn_path[vfs_tok_count(path, '/') + 2];

  vfs_tokenize_path(path, tkn_path, '/');
  node = vfs_load(root_node, tkn_path, &e);

  if (e)
    return NULL;
  //  vfs_print_node(node);
  return (vfs_get_handle(node));
}

error_t vfs_close(struct vfs_handle_s *handle)
{
  assert(handle != NULL);

  vfs_release_handle(handle);

  return 0;
}

struct vfs_handle_s *vfs_opendir(struct vfs_node_s *root_node,
				 char *path,
				 uint_fast16_t mode)
{
  struct vfs_handle_s *handle = vfs_open(root_node, path, mode);

  if (handle != NULL)
    {
      if (handle->fs.ent->flags & FS_ENT_DIRECTORY)
	return handle;
      vfs_close(handle);
    }
  return NULL;
}

error_t vfs_closedir(struct vfs_handle_s *handle)
{
  assert(handle != NULL);

  if (handle->fs.ent->flags & FS_ENT_DIRECTORY)
    return (vfs_close(handle));
  return -1;
}

error_t vfs_find_first_file(struct vfs_handle_s *direntry,
			    struct fs_entity_s *entity,
			    char *filename)
{
  struct vfs_node_s *node = (struct vfs_node_s *)(direntry->fs.ent);

  return (node->ctx->drv->find_first_file(node->ctx->dsk,
					  &direntry->fs,
					  entity,
					  filename));
}

error_t vfs_find_next_file(struct vfs_handle_s *direntry,
 			   struct fs_entity_s *entity,
			   char *filename)
{
  struct vfs_node_s *node = (struct vfs_node_s *)(direntry->fs.ent);

  return (node->ctx->drv->find_next_file(node->ctx->dsk,
					 &direntry->fs,
					 entity, 
					 filename));
}
