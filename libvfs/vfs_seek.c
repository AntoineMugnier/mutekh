#include <vfs/vfs.h>

#define VFS_SEEK_START 0
#define VFS_SEEK_CURR 1
#define VFS_SEEK_END 2

error_t vfs_seek(struct vfs_handle_s *handle,
		 uint32_t size,
		 uint_fast8_t opt)
{
  if (opt > 2 || handle->fs.ent->flags & FS_ENT_DIRECTORY)
    return FS_ERROR_INVALID;

  struct vfs_node_s *node = (struct vfs_node_s *)(handle->fs.ent);

  return node->ctx->drv->read(node->ctx->dsk, &handle->fs, buffer, size);



  switch (opt)
    {
    case VFS_SEEK_START:
      
      break;
    case VFS_SEEK_START:

      break;
    case VFS_SEEK_START:

      break;
    default:
      return FS_ERROR_INVALID;
    }

  return FS_OK;
}
