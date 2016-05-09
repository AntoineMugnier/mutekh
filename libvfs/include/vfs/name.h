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
   @module {Libraries::Virtual File System}
   @short Core operations on file system nodes
 */

#ifndef _VFS_NAME_H_
#define _VFS_NAME_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <vfs/node.h>
#include <vfs/fs.h>


/**
   @this compares a full name as described by the on disk file
   system directory entry with a possibly shortened and mangled node
   name as seen by the vfs.

   @param fullname entry full name as described by file system
   @param fullnamelen lenght of full name
   @param vfsname possibly shortened node name
   @param vfsnamelen possibly shortened node name lenght
   @return true if equal
   @see vfs_name_mangle
 */
bool_t vfs_name_compare(const char *fullname, size_t fullnamelen,
                            const char *vfsname, size_t vfsnamelen);

/**
   @this setup a possibly mangled and shortened vfs node name from a
   full lenght file system entry name. No extra @tt '\0' is added to mangled name.

   @param fullname entry full name as described by file system
   @param fullnamelen lenght of full name
   @param vfsname possibly shortened node name
   @return length of resulting mangled name.
   @see vfs_name_compare @see vfs_node_new
 */
size_t vfs_name_mangle(const char *fullname, size_t fullnamelen, char *vfsname);


C_HEADER_END

#endif
