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

    Based on OSKit
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtld/rtld.h>
#include <rtld/rtld-private.h>

error_t rtld_user_init (void)
{
	_rtld_debug("rtld_user_init\n");

	dynobj_list_init(&user_dynobj_list.prg_root);
	dynobj_list_init(&user_dynobj_list.shobj_root);

    return 0;
}

error_t rtld_user_dlopen (const char *pathname, uintptr_t *entrypoint, void **handle)
{
    dynobj_desc_t *dynobj;

	_rtld_debug("rtld_user_dlopen\n");

	if (_rtld_load_dynobj(pathname, &user_dynobj_list.prg_root, &user_dynobj_list.shobj_root, &dynobj) != 0)
		return -1;

    *entrypoint = dynobj->entrypoint;
    *handle = dynobj;

    return 0;
}

error_t rtld_user_dlsym (const void *handle, const char *name, void **sym)
{
	const dynobj_desc_t *dynobj = handle;
	const elf_sym_t *dlsym;

	_rtld_debug("rtld_user_dlsym\n");

	/* allows to access only loaded programs */
	if (_rtld_lookup_ref(dynobj, &user_dynobj_list.prg_root) == NULL)
		return -1;

	/* look up the symbol in the program 
     * (you must link your program with --export-dynamic if you want to access unreferrenced symbol) 
     */
	if ((dlsym = _rtld_lookup_sym_dynobj(name, _rtld_elf_hash(name), dynobj, ELF_RTYPE_CLASS_PLT)) == NULL)
		return -1;

	/* relocate the symbol */
	*sym = dlsym->st_value + dynobj->relocbase;

	return 0;
}

error_t rtld_user_dlclose (const void *handle)
{
	_rtld_debug("rtld_user_dlclose\n");

    return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

