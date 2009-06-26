/*-
 * Copyright 1996-1998 John D. Polstra.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 1999 University of Utah and the Flux Group.
 * All rights reserved.
 */

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

#include <rtld/rtld-private.h>

static error_t _rtld_partial_load_dynobj(const unsigned char *pathname, 
        dynobj_list_root_t *list_dynobj, 
        dynobj_list_root_t *list_dep, 
        dynobj_desc_t **dynobj);

/* Compute the elf hash on the given name
 *
 * @param name Name to be hashed
 * @return reg_t Hash of the name
 */
reg_t
_rtld_elf_hash(const unsigned char *name)
{
    /*
     * from uClibc 
     */
    reg_t hash=0;
    reg_t tmp;

    while (*name)
    {
        hash = (hash << 4) + *name++;
        tmp = hash & 0xf0000000;
        /* The algorithm specified in the ELF ABI is as follows:
           if (tmp != 0)
               hash ^= tmp >> 24;
           hash &= ~tmp;
           But the following is equivalent and a lot
           faster, especially on modern processors. */
        hash ^= tmp;
        hash ^= tmp >> 24;
    }
    return hash;
}

/* Check if a symbol matches with another
 *
 * @param sym The tried symbol
 * @param sym_name The name of the tried symbol
 * @param name Name of the seeked symbol
 * @param type_class Reloc type classes (cpu dependent)
 * @return bool_t Was there a match?
 */
static bool_t
_rtld_check_match_sym(const elf_sym_t *sym, const unsigned char *sym_name, const unsigned char *name, uint_fast8_t type_class)
{
    _rtld_debug("_rtld_check_match_sym\n");
    _rtld_debug("\tchecking match with symbol \"%s\"\n", sym_name);

    /*
     * from uClibc and glibc
     */
    if (type_class & (sym->st_shndx == SHN_UNDEF))
        /* undefined symbol itself */
        return 0;

    if (sym->st_value == 0 && ELF_ST_TYPE(sym->st_info) != STT_TLS)
        /* No value (accepted for TLS, since offset can be null) */
        return 0;

    if (ELF_ST_TYPE(sym->st_info) > STT_FUNC
            && ELF_ST_TYPE(sym->st_info) != STT_COMMON
            && ELF_ST_TYPE(sym->st_info) != STT_TLS)
        /* Ignore all but STT_NOTYPE, STT_OBJECT, STT_FUNC
         * and STT_COMMON entries (and STT_TLS) since these are no
         * code/data definitions
         */
        return 0;

    if (strcmp((char*)sym_name, (char*)name) != 0)
        return 0;

    /* This is the matching symbol */
    _rtld_debug("\t\tmatch!\n");
    return 1;
}

/* Search a symbol by its name within a given dynamic object
 *
 * @param name Name of the seeked symbol
 * @param hash Hash of the name of the seeked symbol (to avoid reperforming the computation)
 * @param dynobj The dynamic object in which the seeked symbol is looking up
 * @param type_class Reloc type classes (cpu dependent)
 * @return elf_sym_t The reference to the symbol if found, NULL otherwise
 */
const elf_sym_t*
_rtld_lookup_sym_dynobj(const unsigned char *name, const reg_t hash, const dynobj_desc_t *dynobj, uint_fast8_t type_class)
{
    _rtld_debug("_rtld_lookup_sym_dynobj\n");

    _rtld_debug("\tlooking up symbol \"%s\" in object \"%s\"\n", name, dynobj->pathname);

    /* first symbol index in the bucket which matches the hash */
    reg_t sym_idx = dynobj->buckets[hash % dynobj->nbuckets];

    for(; sym_idx != STN_UNDEF; sym_idx = dynobj->chains[sym_idx])
    {
        const elf_sym_t *sym;
        const unsigned char *sym_name;

        /* find the symbol in the symbol table */
        assert(sym_idx < dynobj->nchains);
        sym = dynobj->symtab + sym_idx;

        /* find the symbol name */
        assert(sym->st_name != 0);
        sym_name = dynobj->strtab + sym->st_name;

        /* do we have a match on this symbol? */
        if (_rtld_check_match_sym(sym, sym_name, name, type_class) != 0)
            return sym;
    }

    /* nothing was found in this object */
    return NULL;
}

/* Find the definition of a symbol recursively in the relocation chain
 *
 * @param ref_sym Referrenced symbol
 * @param ref_dynobj The dynamic object referrencing the seeked symbol
 * @param def_sym Symbol we found if any
 * @param def_dynobj The dynamic object the found symbol is defined into if any
 * @param def_weak A symbol has been found but is weak
 * @param chain_dynobj The dynamic object we are looking up in
 * @param type_class Reloc type classes (cpu dependent)
 * @return error_t Error code: -1 is not found, 0 is found
 */
static error_t
_rtld_lookup_sym_chain(const elf_sym_t *ref_sym, const dynobj_desc_t *ref_dynobj,
       const elf_sym_t **def_sym, const dynobj_desc_t **def_dynobj, bool_t *def_weak, 
       const dynobj_desc_t *chain_dynobj,
       uint_fast8_t type_class)
{
    const elf_sym_t *sym;
    const unsigned char *sym_name;
    reg_t sym_hash;

    _rtld_debug("_rtld_lookup_sym_chain\n");

    sym_name = ref_dynobj->strtab + ref_sym->st_name;
    sym_hash = _rtld_elf_hash(sym_name);

    _rtld_debug("\tlooking up symbol \"%s\"\n", sym_name);

    /* look up into the chain object:
     *  only if the chain object is not the referrencing object or 
     *  if the referrencing object is not symbolic
     */
    if (chain_dynobj != ref_dynobj || !ref_dynobj->symbolic)
    {
        sym = _rtld_lookup_sym_dynobj(sym_name, sym_hash, chain_dynobj, type_class);

        if (sym != NULL)
        {
            /* stop scanning only if strong definition */
            if (ELF_ST_BIND(sym->st_info) != STB_WEAK)
            {
                _rtld_debug("\t\tfound and strong\n");

                *def_sym = sym;
                *def_dynobj = chain_dynobj;
                *def_weak = 0;

                return 0;
            }
            /* do not override if a previous weak has already been found */
            else if (*def_weak == 0 && ELF_ST_BIND(sym->st_info) == STB_WEAK)
            {
                _rtld_debug("\t\tfound but weak\n");
                *def_sym = sym;
                *def_dynobj = chain_dynobj;
                *def_weak = 1;
            }
            else
                _rtld_debug("\t\tfound but previous weak definition\n");
        }
        else
            _rtld_debug("\t\tnot found\n");
    }

    /* recurse on the dependencies */
    size_t ndep_shobj;
    for (ndep_shobj = 0; ndep_shobj < chain_dynobj->ndep_shobj; ndep_shobj++)
    {
        /* if strong definition found, stop and return. Else, continue */
        if (_rtld_lookup_sym_chain(ref_sym, ref_dynobj,
                    def_sym, def_dynobj, def_weak, 
                    chain_dynobj->dep_shobj[ndep_shobj], type_class) == 0
                && *def_weak == 0)
            return 0;
    }

    /* nothing was found */
    if (*def_weak == 0)
        return -1;

    /* a weak definition was found */
    return 0;
}

/* Find the definition of a symbol
 *  1/ in the referrencing object if symbolic
 *  2/ launch the search in the relocation chain the referrencing object belongs to
 *
 * @param ref_sym Referrenced symbol
 * @param ref_dynobj The dynamic object referrencing the seeked symbol
 * @param def_sym Symbol we found if any
 * @param def_dynobj The dynamic object the symbol is defined into if any
 * @param root_dynobj The dynamic object from which the relocation chain begins
 * @param type_class Reloc type classes (cpu dependent)
 * @return error_t Error code if any
 */
error_t
_rtld_lookup_sym(const elf_sym_t *ref_sym, const dynobj_desc_t *ref_dynobj,
       const elf_sym_t **def_sym, const dynobj_desc_t **def_dynobj,
       const dynobj_desc_t *root_dynobj,
       uint_fast8_t type_class)
{
    const elf_sym_t *sym;
    const unsigned char *sym_name;
    reg_t sym_hash;

    _rtld_debug("_rtld_lookup_sym\n");

    sym_name = ref_dynobj->strtab + ref_sym->st_name;
    sym_hash = _rtld_elf_hash(sym_name);

    _rtld_debug("\tlooking up symbol \"%s\"\n", sym_name);

    /* if the shared dynamic object was compiled with -Bsymbolic,
     * search the global symbol in the referrencing object itself */
    if (ref_dynobj->symbolic)
    {
        _rtld_debug("\t\tsymbolic flag defined\n");

        sym = _rtld_lookup_sym_dynobj(sym_name, sym_hash, ref_dynobj, type_class);
        if (sym != NULL)
        {
            _rtld_debug("\t\tfound\n");

            *def_sym = sym;
            *def_dynobj = ref_dynobj;

            return 0;
        }
        _rtld_debug("\t\tnot found\n");
    }

    /* else search in the chain, starting from the first dynamic object (exec or kernel)
     *  - that why the order of the libs are important in the linkage stage (ld)
     *  - however, without mmu, we cannot support executables which have linked the same
     *    libs but in different order (the first loaded order will be used for all)!
     */
    /* starting condition for the recursive call */
    *def_sym = NULL;
    *def_dynobj = NULL;
    bool_t def_weak = 0;

    if (_rtld_lookup_sym_chain(ref_sym, ref_dynobj, def_sym, def_dynobj, &def_weak, root_dynobj, type_class) != 0)
    {
        _rtld_debug("\t\tcouldn't find the symbol\n");
        return -1;
    }

    return 0;
}

/* Relocate a given object and its dependencies
 *
 * @param dynobj Object to relocate
 * @param root_dynobj Object which is the root of the relocation chain
 * @param bind_now Allows lazy binding (not implemented though)
 * @return error_t Error code if any
 */
static error_t
_rtld_relocate_dynobj(dynobj_desc_t *dynobj, dynobj_desc_t *root_dynobj, bool_t bind_now)
{
    _rtld_debug("_rtld_relocate_dynobj\n");

    _rtld_debug("\trelocate object \"%s\"\n", dynobj->pathname);

    if (dynobj->relocated)
    {
        /* we assume that if the object has already been relocated, 
         * its dependencies have been too */
        _rtld_debug("\t\talready been relocated\n");
        return 0;
    }

    /* sanity check */
    if (dynobj->nbuckets == 0 || dynobj->nchains == 0 
            || dynobj->buckets == NULL || dynobj->symtab == NULL 
            || dynobj->strtab == NULL)
    {
        _rtld_debug("\tdynobj has not runtime symbol\n");
        return -1;
    }

    /*
     * First relocate the given object 
     */

#if defined(CONFIG_CPU_MIPS)
    /* relocate global entries of GOT (only for mips) */
    if (_rtld_mips_global_got_relocations(dynobj, root_dynobj) != 0)
        return -1;
#endif

    /* Relocate non-plt entries if any */
    if (dynobj->rel)
        if (_rtld_parse_relocations(dynobj, root_dynobj, dynobj->rel, dynobj->relsize) != 0)
            return -1;

    /* Relocate plt entries if any */
    if (dynobj->pltrel)
        if (_rtld_parse_relocations(dynobj, root_dynobj, dynobj->pltrel, dynobj->pltrelsize) != 0)
            return -1;
    
    /* set the object as done */
    dynobj->relocated = 1;

    /*
     * Then relocate the dependencies
     */
    size_t ndep_shobj;
    for (ndep_shobj = 0; ndep_shobj < dynobj->ndep_shobj; ndep_shobj++)
    {
        if (_rtld_relocate_dynobj(dynobj->dep_shobj[ndep_shobj], root_dynobj, bind_now) != 0)
            return -1;
    }
        
    return 0;
}

/* Partially load the dependencies of a given object
 *
 * @param dynobj Object for which load dependencies
 * @param list_dep List of objects to look up into (and to add new object into)
 * @return error_t Error code if any
 */
static error_t
_rtld_load_dependencies(dynobj_desc_t *dynobj, 
        dynobj_list_root_t *list_dep)
{
    _rtld_debug("_rtld_load_dependencies\n");

    const elf_dyn_t *dynp;
    size_t ndep_shobj;

    /* allocate a pointer array to dependencies */
    dynobj->dep_shobj = (dynobj_desc_t**)malloc(dynobj->ndep_shobj*sizeof(dynobj_desc_t*));

    ndep_shobj = 0;
    /* second pass on the dynamic section but just for dependencies */
    for (dynp = dynobj->dynamic; dynp->d_tag != DT_NULL; dynp++)
    {
        switch (dynp->d_tag)
        {
            case DT_NEEDED:
                /* sanity check */
                assert(ndep_shobj < dynobj->ndep_shobj);

                /* find the pathname of the dependency */
                const unsigned char *dep_name = dynobj->strtab + dynp->d_un.d_val;
                dynobj_desc_t *dep_dynobj;

                _rtld_debug("\tload dependency named \"%s\"\n", dep_name);

                /* partially load the dependency into memory */
                if (_rtld_partial_load_dynobj(dep_name, list_dep, list_dep, &dep_dynobj) != 0)
                    return -1;

                _rtld_debug("\tfinished loading dependency named \"%s\"\n", dep_name);

                /* if the dependency uses TLS */
                if (dep_dynobj->tls_nb_modid)
                {
                    dynobj->tls_nb_modid++;
                    if (dep_dynobj->tls_max_modid > dynobj->tls_max_modid)
                        dynobj->tls_max_modid = dep_dynobj->tls_max_modid;
                    _rtld_debug("\tdependency has tls; %d tls for object and max modid is %d\n", dynobj->tls_nb_modid, dynobj->tls_max_modid);
                }
                /* add to the dependencies table */
                dynobj->dep_shobj[ndep_shobj] = dep_dynobj;
                ndep_shobj++;
                break;

            default:
                /* nothing */
                break;
        }
    }

    return 0;
}

/* Process the dynamic section and collect information
 *
 * @param dynobj Object to process
 * @return error_t Error code if any
 */
#if defined(CONFIG_RTLD_DEBUG)
static const char* const _rtld_dyn_tag_names[24] = {
    "DT_NULL", "DT_NEEDED", "DT_PLTRELSZ", "DT_PLTGOT",
    "DT_HASH", "DT_STRTAB", "DT_SYMTAB", "DT_RELA",
    "DT_RELASZ", "DT_RELAENT", "DT_STRSZ", "DT_SYMENT",
    "DT_INIT", "DT_FINI", "DT_SONAME", "DT_RPATH",
    "DT_SYMBOLIC", "DT_REL", "DT_RELSZ", "DT_RELENT",
    "DT_PLTREL", "DT_DEBUG", "DT_TEXTREL", "DT_JMPREL"
};
#endif
static error_t
_rtld_process_dynamic(dynobj_desc_t *dynobj)
{
    _rtld_debug("_rtld_process_dynamic\n");

    const elf_dyn_t *dynp;

    for (dynp = dynobj->dynamic; dynp->d_tag != DT_NULL; dynp++)
    {
        if (dynp->d_tag < 24)
            _rtld_debug("\ttag \'%s\':", _rtld_dyn_tag_names[dynp->d_tag]);
        else
            _rtld_debug("\ttag \'0x%x\':", dynp->d_tag);
        _rtld_debug(" 0x%x\n", dynp->d_un.d_val);

        switch (dynp->d_tag)
        {
            case DT_REL:
            case DT_RELA:
                assert(DT_SUPPORTED_RELOC_TYPE == dynp->d_tag);
                dynobj->rel = (const elf_reloc_t *) (dynobj->relocbase + dynp->d_un.d_ptr);
                break;
            case DT_RELSZ:
            case DT_RELASZ:
                dynobj->relsize = dynp->d_un.d_val;
                break;
            case DT_RELAENT:
            case DT_RELENT:
                assert(dynp->d_un.d_val == sizeof(elf_reloc_t));
                break;
            case DT_JMPREL:
                dynobj->pltrel = (const elf_reloc_t *)(dynobj->relocbase + dynp->d_un.d_ptr);
                break;
            case DT_PLTRELSZ:
                dynobj->pltrelsize = dynp->d_un.d_val;
                break;
            case DT_PLTREL:
                assert(DT_SUPPORTED_RELOC_TYPE == dynp->d_un.d_val);
                break;
            case DT_SYMTAB:
                dynobj->symtab = (const elf_sym_t *)(dynobj->relocbase + dynp->d_un.d_ptr);
                break;
            case DT_SYMENT:
                assert(dynp->d_un.d_val == sizeof(elf_sym_t));
                break;
            case DT_STRTAB:
                dynobj->strtab = (const unsigned char *)(dynobj->relocbase + dynp->d_un.d_ptr);
                break;
            case DT_STRSZ:
                dynobj->strsize = dynp->d_un.d_val;
                break;
            case DT_HASH:
                {
                    const elf_addr_t *hashtab = (const elf_addr_t *)(dynobj->relocbase + dynp->d_un.d_ptr);
                    dynobj->nbuckets = hashtab[0];
                    dynobj->nchains = hashtab[1];
                    dynobj->buckets = &hashtab[2];
                    dynobj->chains = dynobj->buckets + dynobj->nbuckets;
                }
                break;
            case DT_NEEDED:
                _rtld_debug("\t\tNeeded library handled later...\n");
                dynobj->ndep_shobj++;
                break;
            case DT_PLTGOT:
                dynobj->got = (elf_addr_t *)(dynobj->relocbase + dynp->d_un.d_ptr);
                break;

            case DT_TEXTREL:
                _rtld_debug("\t\tnot supported!\n");
                return -1;

            case DT_SYMBOLIC:
                dynobj->symbolic = 1;
                break;

            case DT_RPATH:
                _rtld_debug("\t\tDT_RPATH ignored:\n\
                        \t\t- all needed libs must be in the program directory\n\
                        \t\t- still continuing but it may fail...\n");
                break;

            case DT_SONAME:
                _rtld_debug("\t\tignored!\n");
                break;

            case DT_INIT:
                dynobj->init = (void (*)(void))(dynobj->relocbase + dynp->d_un.d_ptr);
                break;

            case DT_FINI:
                dynobj->fini = (void (*)(void))(dynobj->relocbase + dynp->d_un.d_ptr);
                break;

            case DT_DEBUG:
                _rtld_debug("\t\tignored!\n");
                break;

            case DT_FLAGS:
                if (dynp->d_un.d_val & DF_STATIC_TLS)
                {
                    if (dynobj->program)
                        _rtld_debug("\t\tStatic model for TLS is used for this executable\n");
                    else
                    {
                        _rtld_debug("\t\tStatic model for TLS is not supported for shared libs!\n");
                        return -1;
                    }
                }
                break;

#if defined(CONFIG_CPU_MIPS)
            case DT_MIPS_GOTSYM:
                dynobj->mips_gotsym = dynp->d_un.d_val;
                _rtld_debug("\t\tmips tag DT_MIPS_GOTSYM\n");
                break;
            case DT_MIPS_LOCAL_GOTNO:
                dynobj->mips_local_gotno = dynp->d_un.d_val;
                _rtld_debug("\t\tmips tag DT_MIPS_LOCAL_GOTNO\n");
                break;
            case DT_MIPS_SYMTABNO:
                dynobj->mips_symtabno = dynp->d_un.d_val;
                _rtld_debug("\t\tmips tag DT_MIPS_SYMTABNO\n");
                break;
#endif

            default:
                _rtld_debug("\t\tignored\n");
                break;
        }
    }

    /* 
     * cpu dependent :
     *  - for all, relocate stub addr (in case of lazy binding - not supported here)
     *  - for mips, relocate also local got symbols
     */
    _rtld_debug("\tinit got...\n");
    INIT_GOT(dynobj);

    return 0;
}

/* Look up for a dynamic object from an address
 *
 * @param addr The address at which we look up
 * @param list_lookup List of objects to look up into
 * @return dynobj_desc_t Return the found object if any
 */
static const dynobj_desc_t*
_rtld_lookup_addr(const uintptr_t addr, dynobj_list_root_t *list)
{
#define END_SYM (unsigned char*)"_end"

    _rtld_debug("_rtld_lookup_addr\n");
    _rtld_debug("\tlook up for dynobj at addr %0x\n", addr);

    CONTAINER_FOREACH(dynobj_list, DLIST, list,
    {
        const elf_sym_t *end_sym;

        if (addr < item->mapbase)
            CONTAINER_FOREACH_CONTINUE;
        if ((end_sym = _rtld_lookup_sym_dynobj(END_SYM, _rtld_elf_hash(END_SYM), item, ELF_RTYPE_CLASS_PLT)) != NULL)
        {
            if (addr < (item->relocbase + end_sym->st_value))
            {
                _rtld_debug("\tfound (\"%s\")\n", item->pathname);
                return item;
            }
        }
    });

    _rtld_debug("\tnot found\n");
    return NULL;

#undef END_SYM
}

/* Look up for a dynamic object from its reference
 *
 * @param dynobj Reference of the object
 * @param list_lookup List of objects to look up into
 * @return dynobj_desc_t Return the found object if any
 */
const dynobj_desc_t*
_rtld_lookup_ref(const dynobj_desc_t *dynobj,
        dynobj_list_root_t *list_lookup) 
{
    _rtld_debug("_rtld_lookup_ref\n");

    CONTAINER_FOREACH(dynobj_list, DLIST, list_lookup,
    {
        if (dynobj == item)
        {
            _rtld_debug("\tfound\n");
            return item;
        }
    });

    _rtld_debug("\tnot found\n");
    return NULL;
}

/* Look up for a dynamic object from its name into a given list
 *
 * @param pathname Pathname of the object file
 * @param list_lookup List of objects to look up into
 * @return dynobj_desc_t Return the found object if any
 */
static const dynobj_desc_t*
_rtld_lookup_name(const unsigned char *pathname, 
        dynobj_list_root_t *list_lookup) 
{
#ifdef CONFIG_RTLD_VFSNAME_HACK
    unsigned char dynobj_name[13];
    touppershortname(dynobj_name, pathname);
#else
    unsigned char *dynobj_name=pathname;
#endif

    _rtld_debug("_rtld_lookup_name\n");
    _rtld_debug("\tlooking up for \"%s\" object\n", dynobj_name);

    CONTAINER_FOREACH(dynobj_list, DLIST, list_lookup,
    {
        if (strcmp((char*)dynobj_name, (char*)item->pathname) == 0)
        {
            _rtld_debug("\tfound\n");
            return item;
        }
    });

    _rtld_debug("\tnot found\n");
    return NULL;
}

/* Load partially a dynamic object and its dependencies
 *  1/ the object is already loaded, do nothing
 *  2/ otherwise, load the object and its dependencies but do not relocate
 *
 * @param pathname Pathname of the object file
 * @param list_dynobj List of objects to add or look up the object into
 * @param list_dep List of objects to look up the dependencies into
 * @param dynobj To be set with the reference of the loaded object 
 * @return error_t Error code if any
 */
static error_t
_rtld_partial_load_dynobj(const unsigned char *pathname, 
        dynobj_list_root_t *list_dynobj, 
        dynobj_list_root_t *list_dep, 
        dynobj_desc_t **dynobj)
{
    dynobj_desc_t *new_dynobj;

    _rtld_debug("_rtld_partial_load_dynobj\n");

	/* first, look up by name in already loaded programs */
	new_dynobj = _rtld_lookup_name(pathname, list_dynobj);

	/* if not found, instantiate it completely */
	if (new_dynobj == NULL)
    {
        /* create it */
        new_dynobj = (dynobj_desc_t*)calloc(1, sizeof(dynobj_desc_t));

        if (_elf_load_file(pathname, new_dynobj) != 0)
            goto err;

        /* add the new dynobj at the end of add list */
        dynobj_list_pushback(list_dynobj, new_dynobj);

        if (_rtld_process_dynamic(new_dynobj) != 0)
            goto err;

        /* Load any needed shared objects */
        if (_rtld_load_dependencies(new_dynobj, list_dep) != 0)
        {
            _rtld_debug("\tcould not load dependencies\n");
            goto err_mem;
        }
    }

    new_dynobj->refcount++;
    *dynobj = new_dynobj;
    
    return 0;

err_mem:
    free(new_dynobj->pathname);
err:
    free(new_dynobj);
    return -1;
}

/* Load a dynamic object, its dependencies, and relocate
 *
 * @param pathname Pathname of the program file
 * @param list_prg List of objects to add or look up the program into
 * @param list_dep List of objects to look up the dependencies into
 * @param dynobj To be set with the reference of the loaded object 
 * @param threadpointer Address to be set with the begin of the tls area
 * @return error_t Error code if any
 */
error_t
_rtld_load_dynobj(const unsigned char *pathname, 
        dynobj_list_root_t *list_prg, 
        dynobj_list_root_t *list_dep, 
        dynobj_desc_t **dynobj)
{
    dynobj_desc_t *new_dynobj;

    _rtld_debug("_rtld_load_dynobj\n");

    /* find the object (either by look up or by loading it) */
    if (_rtld_partial_load_dynobj(pathname, list_prg, list_dep, &new_dynobj) != 0)
    {
        _rtld_debug("\tcould not load \"%s\"\n", pathname);
        goto err;
    }

    /* tls management */
    if (_tls_load_dynobj(new_dynobj) != 0)
    {
        _rtld_debug("\tcould not load tls for \"%s\"\n", pathname);
        goto err;
    }

    /* relocate the object */
    if (_rtld_relocate_dynobj(new_dynobj, new_dynobj, 1) != 0)
    {
        _rtld_debug("\tcould not relocate \"%s\"\n", pathname);
        goto err;
    }

    *dynobj = new_dynobj;
    
    return 0;

err:
    return -1;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

