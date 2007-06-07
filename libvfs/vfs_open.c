#include <vfs/vfs.h>

struct fs_handle_s *vfs_open(struct vfs_node_s *root_node,
			     char *path,
			     uint_fast16_t mode)
{
  struct vfs_node_s *node;
  error_t e;
  char *tkn_path[vfs_tok_count(path, '/') + 2];

  vfs_tokenize_path(path, tkn_path, '/');

  node = vfs_load(root_node, tkn_path, &e);

}
