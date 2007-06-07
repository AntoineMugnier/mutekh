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
  struct vfs_node_s *tmp;
  uint16_t idx;

  *ec = 0;
  for (idx = 0; tpath[idx]; idx++)
    {
      if ((target = vfs_get_node(target, tpath[idx])) == NULL)
	{
	  *ec = idx;
	  goto end;
	}
      tmp = target;
    }
 end:
  return tmp;
}
