#include <stdlib.h>
#include <stdio.h>
#include <vfs/vfs.h>

#define DIR_DELIMITER '/'

static void vfs_pv_print_path(uint_fast16_t pos, struct vfs_node_s *node)
{
  static uint_fast8_t ok = 0;
  uint_fast16_t offset = strlen(node->ent.name) + pos + 1;

  if (ok)
    {
      uint_fast16_t e = pos;

      while (e--)
	printf("-");
      ok = 0;
    }

  printf("/%s", node->ent.name);

  CONTAINER_FOREACH(vfs_node_list, CLIST, &node->children,
  {
    vfs_pv_print_path(offset, item);
  });

  if (vfs_node_func_isempty(&node->children) && (ok = 1))
    printf("\n");
}

void vfs_debug_tree(struct vfs_node_s *root_node)
{
  vfs_pv_print_path(0, root_node);
}

uint16_t vfs_tok_count(char *path, char token)
{
  uint16_t idx = 0;
  char *p = path;

  while ((p = strchr(p, token)) != NULL)
    {
      p++;
      idx++;
    }
  return idx;
}

void vfs_tokenize_path(char *path, char *t[], char token)
{
  uint16_t idx = 1;
  char *p = (*path == token) ? path + 1 : path;

  t[0] = p;

  while ((p = strchr(p, token)) != NULL)
    {
      *p = 0;
      t[idx++] = ++p;
    }
  t[idx] = 0;
}

void vfs_print_node(struct vfs_node_s *node)
{
  assert (node != NULL);

  printf("name:\t%s\n", node->ent.name);
  printf("flags:\t%p\n", node->ent.flags);
  printf("size:\t%p\n", node->ent.size);
}
