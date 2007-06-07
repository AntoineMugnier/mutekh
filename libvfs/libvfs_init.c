#include <hexo/alloc.h>
#include <vfs/vfs.h>

error_t vfs_init(struct vfs_node_s *root,
		 const struct vfs_drv_s *drv,
		 struct device_s *device)
{
  assert(root != NULL);
  assert(drv != NULL);

  error_t e;
  struct vfs_context_s *ctx = mem_alloc(sizeof(*ctx), MEM_SCOPE_SYS);

  if (ctx == NULL)
    return -1;

  memset(root, 0x0, sizeof(*root));
  memset(ctx, 0x0, sizeof(*ctx));

  ctx->dsk = drv->context_create(device);
  assert(ctx->dsk != NULL);

  ctx->mpoint = root;
  ctx->drv = drv;

  root->ctx = ctx;

  e = drv->get_root_info(ctx->dsk, &root->ent);
  assert(e == 0);

  vfs_node_func_init(&root->children);

  return 0;
}
