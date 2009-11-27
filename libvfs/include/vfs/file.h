/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.
    
  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
    
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

/**
   @file
   @module {Virtual File System}
   @short Operations on file handles
 */

#ifndef _VFS_FILE_H_
#define _VFS_FILE_H_

#include <hexo/types.h>

enum vfs_open_flags_e {
    /** Allow read operation */
    VFS_OPEN_READ = 1,
    /** Allow write operation */
    VFS_OPEN_WRITE = 2,
    /** Create the file if nonexistant */
    VFS_OPEN_CREATE = 4,
    /** Seek at end of file upon open */
    VFS_OPEN_APPEND = 8,
    /** Open a directory (only valid with VFS_OPEN_READ) */
    VFS_OPEN_DIR = 16,
};

enum vfs_whence_e {
    /** Seek from start of file */
	VFS_SEEK_SET,
    /** Seek from end of file */
	VFS_SEEK_END,
    /** Seek from current point in file */
	VFS_SEEK_CUR,
};

struct vfs_node_s;
struct vfs_file_s;

/** @this defines the file close operation prototype */
#define VFS_FILE_CLOSE(x) error_t (x)(struct vfs_file_s *file)

/**
   @this closes an open file descriptor.  The file descriptor must not
   be used afterwards.

   @param file File descriptor to close
   @return 0 on successful close

   @csee #VFS_FILE_CLOSE
 */
typedef VFS_FILE_CLOSE(vfs_file_close_t);


/** @this defines the file read operation prototype */
#define VFS_FILE_READ(x) ssize_t (x)(struct vfs_file_s *file, void *buffer, size_t size)

/**
   @this reads from a file.

   @param file File descriptor to read from
   @param buffer User buffer to fill
   @param size Size of transfer in bytes
   @return @tt size or less for valid transfers, 0 on end-of-file
   condition, a negative number on error conditions

   @csee #VFS_FILE_READ
 */
typedef VFS_FILE_READ(vfs_file_read_t);

/**
   @this is a stub read function for generic implementation of
   unavailable read operation.

   @csee #VFS_FILE_READ
 */
VFS_FILE_READ(vfs_file_read_na);


/** @this defines the file write operation prototype */
#define VFS_FILE_WRITE(x) ssize_t (x)(struct vfs_file_s *file, const void *buffer, size_t size)

/**
   @this writes to a file.

   @param file File descriptor to write to
   @param buffer User buffer to read data from
   @param size Size of transfer in bytes
   @return @tt size or less for valid transfers, 0 on end-of-file
   condition, a negative number on error conditions

   @csee #VFS_FILE_WRITE
 */
typedef VFS_FILE_WRITE(vfs_file_write_t);

/**
   @this is a stub write function for generic implementation of
   unavailable write operation.

   @csee #VFS_FILE_WRITE
 */
VFS_FILE_WRITE(vfs_file_write_na);


/** @this defines the file seek operation prototype */
#define VFS_FILE_SEEK(x) off_t (x)(struct vfs_file_s *file, off_t offset, enum vfs_whence_e whence)

/**
   @this changes current point into a file.

   @param file File descriptor to seek in
   @param offset Offset to move the point by
   @param whence Reference point to calculate the offset from
   @return the new absolute point from start of file

   @this function may be used to seek beyond end of file. This is not
   an error until another operation is performed.

   @csee #VFS_FILE_SEEK 
 */
typedef VFS_FILE_SEEK(vfs_file_seek_t);

/**
   @this is a stub seek function for generic implementation of
   unavailable seek operation.

   @csee #VFS_FILE_SEEK
 */
VFS_FILE_SEEK(vfs_file_seek_na);


OBJECT_TYPE     (vfs_file, REFCOUNT, struct vfs_file_s);
OBJECT_PROTOTYPE(vfs_file, static inline, vfs_file);

struct vfs_file_s
{
	vfs_file_entry_t obj_entry;
    /** Corresponding node in the VFS */
	struct vfs_node_s *node;
    /** Close operation for this file */
	vfs_file_close_t *close;
    /** Read operation for this file */
	vfs_file_read_t *read;
    /** Write operation for this file */
	vfs_file_write_t *write;
    /** Seek operation for this file */
	vfs_file_seek_t *seek;
    /** Current point in file */
	off_t offset;
    /** File system private data */
	void *priv;
};

struct vfs_dirent_s
{
    /** Name of the directory entry */
	char name[CONFIG_VFS_NAMELEN];
    /** Type of node */
	enum vfs_node_type_e type;
    /** Size of file in bytes, or count of children nodes excluding
        "." and ".." */
	size_t size;
};

OBJECT_CONSTRUCTOR(vfs_file);
OBJECT_DESTRUCTOR(vfs_file);

OBJECT_FUNC   (vfs_file, REFCOUNT, static inline, vfs_file, obj_entry);


/**
   @this reads from an opened file

   @param file File descriptor
   @param buffer Buffer to read into
   @param size Size of the transfer
   @return the size of buffer actually read
*/
static inline ssize_t vfs_file_read(struct vfs_file_s *file,
					  void *buffer,
					  size_t size)
{
	return file->read(file, buffer, size);
}

/**
   @this writes from an opened file

   @param file File descriptor
   @param buffer Buffer to write into
   @param size Size of the transfer
   @return the size of buffer actually written
*/
static inline ssize_t vfs_file_write(struct vfs_file_s *file,
					   const void *buffer,
					   size_t size)
{
	return file->write(file, buffer, size);
}

/**
   @this closes an opened file

   @param file File descriptor to close
   @return 0 if closed correctly
*/
static inline error_t vfs_file_close(struct vfs_file_s *file)
{
	return file->close(file);
}

/**
   @this seeks to the given position into an opened file

   @param file File descriptor to seek into
   @param offset To seek from given point
   @param whence Reference point to seek from
   @return new absolute position from the beginning of file
*/
static inline off_t vfs_file_seek(struct vfs_file_s *file,
					  off_t offset,
					  enum vfs_whence_e whence)
{
	return file->seek(file, offset, whence);
}

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

