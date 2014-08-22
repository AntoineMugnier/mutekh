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

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009,2014
*/

/**
   @file
   @module {Virtual File System}
   @short Operations on file handles

   @section {The node_open operation}
   @alias node_open
   
   When opening a node, @ref vfs_open_flags_e are passed to tell the
   VFS and the filesystem driver what type of operation is required.

   When opening directories, the only valid operation is to open
   read-only. Thus the correct flags are: @ref VFS_OPEN_READ @tt{|}
   @ref VFS_OPEN_DIR.

   When opening files, one may open with @ref VFS_OPEN_READ and/or
   @ref VFS_OPEN_WRITE. If @ref VFS_OPEN_APPEND is passed-in, @tt
   write operations will always write at the end of file.

   @ref VFS_OPEN_CREATE is only valid for creating files (so it is
   invalid with @ref VFS_OPEN_DIR), and is not supported at all layers
   of the VFS. It is valid when using @ref vfs_open, but invalid when
   using @tt node_open action of @ref vfs_fs_ops_s (because node
   already exists).
   
   @ref VFS_OPEN_TRUNCATE is to erase contents of a file at opening,
   filesystem is responsible for implementing this flag.
   
   @end section

 */

#ifndef _VFS_FILE_H_
#define _VFS_FILE_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <vfs/defs.h>

#include <gct/refcount.h>

enum vfs_open_flags_e
{
    /** Allow read operation */
    VFS_OPEN_READ = 1,
    /** Allow write operation */
    VFS_OPEN_WRITE = 2,
    /** Create the file if nonexistant */
    VFS_OPEN_CREATE = 4,
    /** Always write at end */
    VFS_OPEN_APPEND = 8,
    /** Erase contents of file on open, this is filesystem-implemented */
    VFS_OPEN_TRUNCATE = 16,
    /** Open a directory (only valid with VFS_OPEN_READ) */
    VFS_OPEN_DIR = 32,
};

/** Compatible with @ref seek_whence_e */
enum vfs_whence_e
{
    /** Seek from start of file */
	VFS_SEEK_SET,
    /** Seek from end of file */
	VFS_SEEK_END,
    /** Seek from current point in file */
	VFS_SEEK_CUR,
};

struct vfs_node_s;
struct vfs_file_s;

/** @this defines the file read operation prototype.
    Compatible with @ref #FILEOPS_READ */
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


/** @this defines the file write operation prototype.
    Compatible with @ref #FILEOPS_WRITE */
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



/** @this defines the file seek operation prototype.
    Compatible with @ref #FILEOPS_LSEEK */
#define VFS_FILE_SEEK(x) off_t (x)(struct vfs_file_s *file, off_t offset, enum vfs_whence_e whence)

/**
   @this changes current point into a file.

   @param file File descriptor to seek in
   @param offset Offset to move the point by
   @param whence Reference point to calculate the offset from
   @return the new absolute point from start of file

   @this function may be used to seek beyond end of file. This is not
   an error.

   @csee #VFS_FILE_SEEK 
 */
typedef VFS_FILE_SEEK(vfs_file_seek_t);


/** @this defines the file truncate operation prototype */
#define VFS_FILE_TRUNCATE(x) off_t (x)(struct vfs_file_s *file, off_t new_size)

/**
   @this changes size of a file to exactly @tt new_size

   @param file File to truncate
   @param new_size Offset to cut at
   @return 0 if done

   @this function may be used to truncate beyond end of file. This
   will extent the file with zeros.

   @csee #VFS_FILE_TRUNCATE 
 */
typedef VFS_FILE_TRUNCATE(vfs_file_truncate_t);


/** @This defines the VFS file cleanup prototype */
#define VFS_FILE_CLEANUP(x) void (x) (struct vfs_file_s *file)

/** @This free resources associated with a fs file. */
typedef VFS_FILE_CLEANUP(vfs_file_cleanup_t);

struct vfs_file_ops_s
{
	vfs_file_read_t *read;              //< Read operation for this file   
	vfs_file_write_t *write;            //< Write operation for this file  
	vfs_file_seek_t *seek;              //< Seek operation for this file   
	vfs_file_truncate_t *truncate;      //< Truncate operation for this file   
	vfs_file_cleanup_t *cleanup;        //< Cleanup operation for this file, optional
};

struct vfs_file_s
{
    GCT_REFCOUNT_ENTRY(obj_entry);
	struct vfs_node_s *node;            //< Corresponding node in the FS
	off_t offset;                       //< Current access position in file
    enum vfs_open_flags_e flags;
    const struct vfs_file_ops_s *ops;
};

GCT_REFCOUNT(vfs_file, struct vfs_file_s *, obj_entry);

/** @This initializes common fields of a file object. */
error_t vfs_file_init(struct vfs_file_s *file,
	                  const struct vfs_file_ops_s *ops,
                      enum vfs_open_flags_e flags,
                      struct vfs_node_s *node);

/** @This frees resources allocated by @ref vfs_file_init. */
void vfs_file_cleanup(struct vfs_file_s *file);

/** @This calls the @ref vfs_file_s::close and the @ref
    vfs_file_cleanup functions then free the file object. This is
    called when the file refcount reaches 0. */
void vfs_file_destroy(struct vfs_file_s *file);


/** @this increases the file reference count and return the file itself. */
struct vfs_file_s * vfs_file_refinc(struct vfs_file_s * file);

/** @this decreases the file reference count and may delete the file
    if no more reference exist. */
bool_t vfs_file_refdec(struct vfs_file_s * file);

/**
   @this reads from an opened file

   @param file File descriptor
   @param buffer Buffer to read into
   @param size Size of the transfer
   @return the size of buffer actually read
*/
ssize_t vfs_file_read(struct vfs_file_s *file,
					  void *buffer, size_t size);

/**
   @this writes from an opened file

   @param file File descriptor
   @param buffer Buffer to write into
   @param size Size of the transfer
   @return the size of buffer actually written
*/
ssize_t vfs_file_write(struct vfs_file_s *file,
					   const void *buffer,
					   size_t size);

/**
   @this closes an opened file.
   @param file File descriptor to close
*/
void vfs_file_close(struct vfs_file_s *file);

/**
   @this seeks to the given position into an opened file

   @param file File descriptor to seek into
   @param offset To seek from given point
   @param whence Reference point to seek from
   @return new absolute position from the beginning of file
*/
off_t vfs_file_seek(struct vfs_file_s *file,
                    off_t offset,
					enum vfs_whence_e whence);

/**
   @this truncates the file to the exact size @tt new_size

   @param file File to truncate
   @param new_size New file size
   @return 0 if done
*/
off_t vfs_file_truncate(struct vfs_file_s *file,
					    off_t new_size);

C_HEADER_END

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

