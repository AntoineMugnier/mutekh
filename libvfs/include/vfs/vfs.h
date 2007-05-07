#include <hexo/error.h>
#include <hexo/types.h>

struct fs_file_s
{
  char			*name;
};

struct vfs_fs_s
{
  struct fs_file_s	*(*get_file_info)(struct fs_file_s *parent, char *name);
  int_fast32_t		(*open)();
  void			(*close)();
  int_fast32_t		(*read)();
  int_fast32_t		(*write)();
  /* ... */
};

struct vfs_fs_inst_s
{
  struct vfs_node_s	*mpoint;
  struct vfs_fs_s	*drv;
  struct vfs_node_s	*tmp_chld;
  struct vfs_fs_inst_s	*tmp_fs_inst;
  /* ... */
};

struct vfs_node_s
{
  uint_fast32_t		type;
  struct fs_file_s	*file;
  struct vfs_fs_inst_s	*fs_inst;
  struct vfs_node_s	*parent;
  struct vfs_node_s	*next;
  uint_fast8_t		refcount;
  struct vfs_node_s	*children;
  struct vfs_node_s	*last_child;
  /* ... */
};

extern struct vfs_node_s vfs_root_g;

/* node.c */
struct vfs_node_s *vfs_get_node(struct vfs_node_s *parent, char *name);
void vfs_dbg_print_node(char *str, struct vfs_node_s *node);

/* libVFS.c */
void vfs_parse(char *str);
void vfs_debug_tree(void);

/* libVFS_init.c */
error_t vfs_init(void);
