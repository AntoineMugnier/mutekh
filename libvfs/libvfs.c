#include <stdlib.h>
#include <stdio.h>
#include <vfs/vfs.h>

#define DIR_DELIMITER '/'

static void vfs_pv_print_path(uint_fast16_t pos, struct vfs_node_s *node)
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

  CONTAINER_FOREACH(vfs_node_list, CLIST, &node->children,
  {
    vfs_pv_print_path(offset, item);
  });

  if (vfs_node_func_isempty(&node->children) && (ok = 1))
    printf("\n");
}

/*
** vfs-debug_tree
**
*/

void vfs_debug_tree(void)
{
  vfs_pv_print_path(0, &vfs_root_g);
}

static inline struct vfs_node_s *vfs_get_root_node(void)
{
  return &vfs_root_g;
}

/*
static inline struct vfs_node_s *get_cwd_node()
{

}
*/

/*
** vfs_parse: automates the vfs cache updates.
**
** params
** ------
** @param str	string pointer to a path name.
** @param ec	error code.
**
** description
** -----------
** Tokenize the given path.
** for each token, checks if there is a corresponding entry in the vfs cache.
** if not, it tries to create a node object for that token, and updates the cache.
** (see definition of vfs_get_node).
**
** return types
** ------------
** on success:	pointer to the last token's node, ec = 0.
** on error:	pointer to the last valid node, ec = error code.
**
** 
*/

struct vfs_node_s *vfs_parse(char *str, error_t *ec)
{
  uint_fast16_t i = 0;
  char *ps;
  struct vfs_node_s *target;
  struct vfs_node_s *tmp;

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
	  tmp = target;
          if ((target = vfs_get_node(target, ps)))
	    ps = &str[i];
	  else
	    {
	      *ec = 1;
	      return tmp;
	    }
        }
    }
  if (*ps)
    {
      tmp = target;
      if ((target = vfs_get_node(target, ps)) == NULL)
	{
	  *ec = 1;
	  return tmp;
	}
    }
  *ec = 0;
  return target;
}
