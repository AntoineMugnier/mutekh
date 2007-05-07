#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include <vfs/vfs.h>

static inline void pv_vfs_append_child(struct vfs_node_s *parent, struct vfs_node_s *node)
{
  if (parent->children)
    parent->last_child->next = node;
  else
    parent->children = node;
  parent->last_child = node;
}

static inline void pv_vfs_remove_child(struct vfs_node_s *parent, struct vfs_node_s *node)
{
  struct vfs_node_s *tmp;

  for (tmp = parent->children; tmp; tmp = tmp->next)
    {
      if (tmp->next == node)
	{
	  tmp->next = node->next;
	  if (!node->next)
	    parent->last_child = tmp;
	  return;
	}
    }
}

static inline struct vfs_node_s *pv_vfs_lookup_child(struct vfs_node_s *node, char *name)
{
  struct vfs_node_s *tmp;

  for (tmp = node->children; tmp; tmp = tmp->next)
    {
      if (!strcmp(tmp->file->name, name))
	  return tmp;
    }
  return NULL;
}

static inline void pv_vfs_delete_node(struct vfs_node_s *node)
{
  if (node->parent)
    {
      pv_vfs_remove_child(node->parent, node);
      mem_free(node);
    }
}

static inline void pv_vfs_increase_node_weight(struct vfs_node_s *node)
{
  struct vfs_node_s *tmp;

  for (tmp = node->parent; tmp; tmp = tmp->parent)
      tmp->refcount++;
}

static inline void pv_vfs_decrease_node_weight(struct vfs_node_s *node)
{
  struct vfs_node_s *tmp = node->parent;

  while (tmp)
    {
      tmp->refcount--;
      if (tmp->refcount > 1)
	{
	  tmp = tmp->parent;
	  continue;
	}
      else
	{
	  struct vfs_node_s *t = tmp->parent;

	  pv_vfs_delete_node(tmp);
	  tmp = t;
	}
    }
}

static inline struct vfs_node_s *pv_vfs_create_node(struct vfs_node_s *parent, char *name)
{
  if (parent)
    {
      struct fs_file_s *file;

      file = parent->fs_inst->drv->get_file_info(parent->file, name);
      if (file)
	{
	  struct vfs_node_s *node = mem_alloc(sizeof(*node), MEM_SCOPE_SYS);

	  memset(node, 0x0, sizeof(*node));
	  node->refcount = 1;
	  node->file = file;
	  node->parent = parent;
	  node->fs_inst = parent->fs_inst;
	  pv_vfs_append_child(node->parent, node);
	  pv_vfs_increase_node_weight(node);

	  return node;
	}
    }
  return NULL;
}

inline struct vfs_node_s *vfs_get_node(struct vfs_node_s *parent, char *name)
{
  struct vfs_node_s *node;

  if (!(node = pv_vfs_lookup_child(parent, name)))
    {
      if ((node = pv_vfs_create_node(parent, name)))
	{
	  return node;
	}
      return NULL;
    }
  return node;
}

void vfs_dbg_print_node(char *str, struct vfs_node_s *node)
{
  printf("-node-------\n");
  printf("%s\n", str);
  printf("node:\t(0x%08x) %d\n", node, node->type);
  printf("file:\t(0x%08x) %s\n", node->file, node->file->name);
  printf("fs_int:\t(0x%08x)\n", node->fs_inst);
  printf("parent:\t(0x%08x)\n", node->parent);
  printf("next:\t(0x%08x)\n", node->next);
  printf("refcnt:\t%d\n", node->refcount);
  printf("childn:\t(0x%08x)\n", node->children);
  printf("lastch:\t(0x%08x)\n", node->last_child);
  printf("-end-node---\n");
}
