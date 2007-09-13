#include <vfs/vfs.h>

error_t vfs_read(struct vfs_handle_s *handle,
		 uint8_t *buffer,
		 uint32_t size)
{
  if (handle->fs.ent->flags & FS_ENT_DIRECTORY)
    return FS_ERROR_INVALID;

  struct vfs_node_s *node = (struct vfs_node_s *)(handle->fs.ent);

  return node->ctx->drv->read(node->ctx->dsk, &handle->fs, buffer, size);
}
