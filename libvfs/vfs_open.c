#include <vfs/vfs.h>

struct vfs_node_s *vfs_open(char *path, uint_fast16_t mode)
{
  struct vfs_node_s *node;

  node = vfs_parse(path);
  
}
