#ifndef CWD_H_
#define CWD_H_

#include <hexo/local.h>
#include <vfs/node.h>

extern CONTEXT_LOCAL struct vfs_node_s *_vfs_cwd;
extern struct vfs_node_s *root;

static inline struct vfs_node_s *vfs_get_root(void)
{
    return root;
}

static inline void vfs_set_cwd(struct vfs_node_s *cwd)
{
    struct vfs_node_s *old = CONTEXT_LOCAL_GET(_vfs_cwd);

    CONTEXT_LOCAL_SET(_vfs_cwd, cwd ? vfs_node_refinc(cwd) : NULL);

    if (old)
        vfs_node_refdec(old);
}

static inline struct vfs_node_s *vfs_get_cwd()
{
    if (CONTEXT_LOCAL_GET(_vfs_cwd) == NULL)
        vfs_set_cwd(vfs_get_root());

    return CONTEXT_LOCAL_GET(_vfs_cwd);
}

#endif
