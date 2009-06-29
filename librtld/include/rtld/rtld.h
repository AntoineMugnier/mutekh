/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright (c) UPMC, Lip6, STMicroelectronics
        Joel Porquet <joel.porquet@lip6.fr>, 2009

*/

#ifndef _RTLD_H_
#define _RTLD_H_

#include <hexo/types.h>
#include <hexo/error.h>

/* 
 * Public API 
 */

/* 
 * User program management
 */
typedef struct rtld_map_s {
    uintptr_t   base;
    size_t      size;
    struct rtld_map_s   **deps;
} rtld_map_t;

/* 
 * Initialize the internal rtld library for user purpose
 * Warning: you have to call this function only once (not for each uses)
 */
error_t rtld_user_init (void);

/* Load an user program from a file
 *
 * @param pathname Pathname of the program file
 * @param entrypoint Address of the entrypoint 
 * 					 (to be given as start address for context creation)
 * @param handle Handle on the program object
 * @return error_t Error code if any
 */
error_t rtld_user_dlopen (const unsigned char *pathname, uintptr_t *entrypoint, void **handle);

/* Retrieve a symbol from an user program
 *
 * @param handle Handle on the program object
 * @param name Name of the searched symbol
 * @param sym Handle on the symbol (address where the symbol is loaded)
 * @return error_t Error code if any
 */
error_t rtld_user_dlsym (const void *handle, const unsigned char *name, uintptr_t *sym);

/*
 * Create a tls area for an user program.
 * Note that the caller must take care of freeing this area.
 *
 * @param handle Handle on the program object
 * @param tls Tls area (allocate one if NULL)
 * @param threadpointer Threadpointer for the cpu
 * @return error_t Error code if any
 */
error_t rtld_user_dltls (const void *handle, uintptr_t *tls, uintptr_t *threadpointer);

/* get the size of the tls area for a given object */
error_t rtld_user_tls_size (const void *handle, size_t *size);
/* get the list of the objects mapping */
error_t rtld_user_dl_map (const void* handle, rtld_map_t **maplist);

/* Close an user program
 *
 * @param handle Handle on the program object
 * @return error_t Error code if any
 */
error_t rtld_user_dlclose (const void *handle);


#if defined (CONFIG_RTLD_KERNEL)

/* 
 * Kernel program management
 */

/* 
 * Kernel library management
 */

/* Initialize the internal rtld library for kernel purpose
 *  The goal here is to retrieve the symbol table. For that, either
 *  the loaded kernel has already a symbol table (_DYNAMIC[]) or 
 *  you can give the pathname of the kernel image.
 *
 * @param pathname Pathname of the kernel image file (optional)
 * @return error_t Error code if any
 */
error_t rtld_kernel_init (const unsigned char *pathname);

/* Load an kernel library from a file
 *
 * @param pathname Pathname of the library file
 * @param handle Handle on the program object
 * @return error_t Error code if any
 */
error_t rtld_kernel_dlopen (const unsigned char *pathname, void **handle);

/* Retrieve a symbol from a kernel library
 *
 * @param handle Handle on the library object
 * @param name Name of the searched symbol
 * @param sym Handle on the symbol (address where the symbol is loaded)
 * @return error_t Error code if any
 */
error_t rtld_kernel_dlsym (const void *handle, const unsigned char *name, void **sym);

/* Close a kernel library
 *
 * @param handle Handle on the kernel library
 * @return error_t Error code if any
 */
error_t rtld_kernel_dlclose (const void *handle);

#endif

#endif /* _RTLD_H_ */

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

