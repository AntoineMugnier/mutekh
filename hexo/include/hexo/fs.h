#ifndef __FS_H_
#define __FS_H_

#include <hexo/types.h>

typedef uint64_t fs_file_sz_t;
typedef uint64_t fs_file_off_t;

struct fs_handle_s
{
  struct fs_entity_s	*ent;
  void			*h_pv;
};

struct fs_disk_context_s
{
  struct device_s *device;
};

struct fs_entity_s
{
  char		*name;	/* node name */
  fs_file_sz_t	size;
  uint32_t	flags;	/* type and modes for the node */
  void		*e_pv;	/* driver specific data */
};

#define FS_ENT_UNKNOWN	       0
#define FS_ENT_DIRECTORY       1
#define FS_ENT_FILE            2
#define FS_ENT_VIRTUAL         4

#endif
