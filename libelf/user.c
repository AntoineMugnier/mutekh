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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libelf/rtld.h>

static dynobj_list_root_t prgs_root;    /* List of user programs */
static dynobj_list_root_t deps_root;    /* List of dependencies (shared libs) */

/* Private RTLD prototypes */
error_t _rtld_load_dynobj(struct dynobj_rtld_s **dynobj, const char *pathname,
        dynobj_list_root_t *list_prg, dynobj_list_root_t *list_dep);

const elf_sym_t* _rtld_lookup_sym_dynobj(const struct dynobj_rtld_s *dynobj,
        const char *name, const reg_t hash, uint_fast8_t type_class);

const struct dynobj_rtld_s* _rtld_lookup_ref(const struct dynobj_rtld_s *dynobj, dynobj_list_root_t *list_lookup);

reg_t _rtld_elf_hash(const char *name);


error_t rtld_init (void)
{
	_libelf_debug(DEBUG, "rtld_init\n");
	dynobj_list_init(&prgs_root);
	dynobj_list_init(&deps_root);
    return 0;
}

error_t rtld_configure_callbacks (libelf_alloc_segments_t *alloc_segs,
        libelf_alloc_tls_t *alloc_tls,
        libelf_scan_segments_t *scan_segs,
        void *priv_data)
{
    /* same private data for everyone! */
    /* elf */
    if (alloc_segs) {
        segs_alloc_ctxt.fcn_alloc_segments = alloc_segs;
        segs_alloc_ctxt.priv_data = priv_data;
    }

    /* tls */
    if (alloc_tls) {
        tls_alloc_ctxt.fcn_alloc_tls = alloc_tls;
        tls_alloc_ctxt.priv_data = priv_data;
    }

    /* post scan */
    if (scan_segs) {
        segs_scan_ctxt.fcn_scan_segs = scan_segs;
        segs_scan_ctxt.priv_data = priv_data;
    }

    return 0;
}

error_t rtld_open (struct dynobj_rtld_s **dynobj, const char *pathname)
{
	_libelf_debug(DEBUG, "rtld_open\n");

    struct dynobj_rtld_s *new_dynobj;
	if (_rtld_load_dynobj(&new_dynobj, pathname, &prgs_root, &deps_root) != 0)
		return -1;
    *dynobj = new_dynobj;
    return 0;
}

error_t rtld_sym (const struct dynobj_rtld_s *dynobj, const char *name, uintptr_t *sym)
{
	_libelf_debug(DEBUG, "rtld_sym\n");

	const elf_sym_t *dlsym;

	/* allows to access only loaded programs */
	if (_rtld_lookup_ref(dynobj, &prgs_root) == NULL)
		return -1;
	/* look up the symbol in the program 
     * (note that you should link your program with --export-dynamic 
     * if you want to access unreferenced symbol) 
     */
	if ((dlsym = _rtld_lookup_sym_dynobj(dynobj, name, _rtld_elf_hash(name), ELF_RTYPE_CLASS_PLT)) == NULL)
		return -1;
	/* relocate the symbol */
	*sym = (uintptr_t)(dlsym->st_value + dynobj->elf.relocbase);
	return 0;
}

error_t rtld_tls (const struct dynobj_rtld_s *dynobj, uintptr_t *tls, uintptr_t *threadpointer)
{
	_libelf_debug(DEBUG, "rtld_tls\n");

	/* allows to access only loaded programs */
	if (_rtld_lookup_ref(dynobj, &prgs_root) == NULL)
		return -1;
    /* allocate area */
    if (_tls_allocate_dynobj(dynobj, tls) != 0)
        return -1;
    if (_tls_init_dynobj(dynobj, *tls, threadpointer) != 0)
        return -1;
	return 0;
}

error_t rtld_close (const struct dynobj_rtld_s *dynobj)
{
    //TODO
	_libelf_debug(DEBUG, "rtld_user_dlclose\n");
    return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

