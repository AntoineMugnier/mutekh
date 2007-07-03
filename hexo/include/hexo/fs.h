#ifndef __FS_H_
#define __FS_H_

#include <hexo/types.h>

typedef uint64_t fs_file_sz_t;
typedef uint64_t fs_file_off_t;

struct fs_handle_s
{
  struct fs_entity_s	*ent;
  void			*pv;
};

struct fs_disk_context_s
{
  struct device_s *device;
};

struct fs_entity_s
{
  uint32_t	flags;	/* type and modes for the file */
  char		*name;	/* file name */
  fs_file_sz_t	size;	/* file size */
  void		*pv;	/* driver specific data */
};

#define FS_ENT_VALID_BIT	0x80000000
#define FS_ENT_UNKNOWN		0
#define FS_ENT_DIRECTORY	1
#define FS_ENT_FILE		2
#define FS_ENT_VIRTUAL		4

#endif
