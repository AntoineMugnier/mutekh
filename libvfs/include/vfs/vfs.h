#include <hexo/error.h>
#include <hexo/types.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>

#include <gpct/cont_clist.h>


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

//CONTAINER_PROTOTYPE(vfs_node_list, static, vfs_node_func);

CONTAINER_TYPE(vfs_node_list, CLIST, struct vfs_node_s
{
  CONTAINER_ENTRY_TYPE(CLIST)	list_entry;
  uint_fast32_t			type;
  struct fs_file_s		*file;
  struct vfs_fs_inst_s		*fs_inst;
  struct vfs_node_s		*parent;
  uint_fast8_t			refcount;
  vfs_node_list_root_t		children;
}, list_entry);

CONTAINER_FUNC(vfs_node_list, CLIST, static inline, vfs_node_func);

extern struct vfs_node_s vfs_root_g;

/* node.c */
struct vfs_node_s *vfs_get_node(struct vfs_node_s *parent, char *name);
void vfs_dbg_print_node(char *str, struct vfs_node_s *node);

/* libVFS.c */
void vfs_parse(char *str);
void vfs_debug_tree(void);

/* libVFS_init.c */
error_t vfs_init(void);
