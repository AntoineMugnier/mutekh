#include <vfs/vfs.h>

/**
  vfs_load: automates the vfs cache updates.

  @param [IN]  root_node	starting node
  @param [IN]  tpath		holds the path
  @param [OUT] ec		error code

  function behaviour:
  -------------------
  on success, vfs_load() returns a pointer
  to the last created node, ec is set to 0.

  on error, vfs_load() returns a pointer to
  the last valid node, ec > 0.

  Error Code:
  -----------
  (ec - 1) can serve as an index in tpath in order
  to retreive the first erroneous filename.
  if ec > 2 (eg. root is valid), then (ec - 2) indexes
  the last valid filename.
**/

struct vfs_node_s *vfs_load(struct vfs_node_s *root_node,
			    char *tpath[],
			    error_t *ec)
{
  struct vfs_node_s *target = root_node;
  uint16_t idx;

  *ec = 0;

  // refnew on taget
  vfs_node_obj_func_refnew(target);

  /* get reference on root */
  if (*(tpath[0]) == 0)
    {
      /* no path given, return root */
      printf("no path given, returning root node ...\n");
      return root_node;
    }

  for (idx = 0; tpath[idx]; idx++)
    {
      struct vfs_node_s *next;

      next = vfs_get_node(target, tpath[idx]);

      // drop ref on target
      vfs_node_obj_func_refdrop(target);

      target = next;

      if (next == NULL)
	{
	  *ec = idx;
	  return NULL;
	}
    }
  return target;	/* caller has node ownership */
}
