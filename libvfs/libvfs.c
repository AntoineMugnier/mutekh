#include <stdlib.h>
#include <stdio.h>
#include <vfs/vfs.h>

#define DIR_DELIMITER '/'

void pv_vfs_print_path(uint_fast16_t pos, struct vfs_node_s *node)
{
  struct vfs_node_s *tmp;
  static uint_fast8_t ok = 0;
  uint_fast16_t offset = strlen(node->file->name) + pos + 1;

  if (ok)
    {
      uint_fast16_t e = pos;

      while (e--)
	printf("-");
      ok = 0;
    }

  printf("/%s", node->file->name);
  for (tmp = node->children; tmp; tmp = tmp->next)
    pv_vfs_print_path(offset, tmp);
  if (!node->children && (ok = 1))
    printf("\n");
}

void vfs_debug_tree(void)
{
  pv_vfs_print_path(0, &vfs_root_g);
}

static inline struct vfs_node_s *vfs_get_root_node(void)
{
  printf("rootnode: %s\n", vfs_root_g.file->name);
  return &vfs_root_g;
}

/*
static inline struct vfs_node_s *get_cwd_node()
{

}
*/

void vfs_parse(char *str)
{
  uint_fast16_t i = 0;
  char *ps;
  struct vfs_node_s *target;

  if (str[0] == DIR_DELIMITER)
    {
      /* absolute path */
      while(str[++i] == DIR_DELIMITER)
        ;
      target = vfs_get_root_node();
    }
  else
    {
      /* relative path ... use vfs_get_cwd_node() */
      target = vfs_get_root_node();
    }

  ps = &str[i];

  while (str[i++])
    {
      if (str[i] == DIR_DELIMITER)
        {
          for (;str[i] == DIR_DELIMITER; str[i++] = 0)
            ;
          target = vfs_get_node(target, ps);
          ps = &str[i];
        }
    }
  if (*ps)
    target = vfs_get_node(target, ps);
}
