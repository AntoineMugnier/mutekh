#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include <vfs/vfs.h>

static inline struct vfs_node_s *vfs_lookup_child(struct vfs_node_s *node,
						  char *name)
{
  assert(node != NULL);

  CONTAINER_FOREACH(vfs_node_list, CLIST, &node->children,
  {
    if (!strcmp(item->ent.name, name))
      return item;
  });
  return NULL;
}

static inline void vfs_delete_node(struct vfs_node_s *node)
{
  assert(node->parent != NULL);

  vfs_node_func_remove(&node->parent->children, node);
  vfs_node_func_destroy(&node->children);
  mem_free(node);
}

static inline struct vfs_node_s *vfs_create_node(struct vfs_node_s *parent,
						 char *name)
{
  assert(parent != NULL);
  assert(parent->ent.flags & FS_ENT_DIRECTORY);

  struct fs_entity_s ent;
  error_t e;

  e = parent->ctx->drv->get_entity_info(parent->ctx->dsk,
					&parent->ent,
					&ent,
					name);
  if (!e)
    {
      struct vfs_node_s *node = mem_alloc(sizeof(*node), MEM_SCOPE_SYS);

      //      memset(node, 0x0, sizeof(*node));

      node->parent = parent;
      node->ctx = parent->ctx;

      vfs_node_func_init(&node->children);
      vfs_node_func_push(&parent->children, node);

      memcpy(&node->ent, &ent, sizeof(ent));

      return node;
    }

  return NULL;
}

/*
** vfs_get_node: assuming a valid parent, tries to find a corresponding
**		 node for a file name.
**
** params
** ------
** @param parent	pointer to the parent node.
** @param name		string pointer to a file name.
**
** description
** -----------
** If the file isn't already on the node cache, tries to associate the
** filename with a new cache entry.
**
** return types
** ------------
** on success:	pointer to the vfs node of the file.
** on error:	NULL.
*/

struct vfs_node_s *vfs_get_node(struct vfs_node_s *parent, char *name)
{
  assert(parent != NULL);

  struct vfs_node_s *node;

  if (!(node = vfs_lookup_child(parent, name)))
    node = vfs_create_node(parent, name);
  return node;
}
