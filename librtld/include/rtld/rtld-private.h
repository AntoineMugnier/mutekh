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
 *
 *      $\Id: rtld.h,v 1.1 1999/02/19 15:13:27 stoller Exp stoller $
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

#ifndef _RTLD_PRIVATE_H_
#define _RTLD_PRIVATE_H_

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_dlist.h>

#include <rtld/rtld-types.h>
#include <cpu/tls.h>

/*
 * Utils
 */

#if defined(CONFIG_LIBRTLD_DEBUG)
# define _rtld_debug printk
#else
# define _rtld_debug(...)
#endif

/*
 * Dynamic object descriptor.
 */

#define RTLD_TEXT_SEG    0
#define RTLD_DATA_SEG    1
#define RTLD_TLS_SEG     2

typedef struct dynobj_desc_s
{
    /* 
     * Misc informations 
     */
    unsigned char   *pathname;  /* Pathname of underlying file (%) */
    uint_fast32_t   refcount;
    bool_t          program;    /* True if this is a program/kernel (contrary to a shared lib)*/
    bool_t          symbolic;   /* True if generated with "-Bsymbolic" */
    bool_t          relocated;  /* True if the object is finished being loaded */

    /*
     * Relative to elf loading
     */
    uintptr_t   mapbase;    /* Base address of mapped region */
    size_t      mapsize;    /* Size of mapped region in bytes */

    elf_addr_t  vaddrbase;  /* Base address in shared object file */
    ptrdiff_t   relocbase;  /* Relocation constant = mapbase - vaddrbase (can be negative)*/

    const elf_dyn_t *dynamic;   /* Dynamic section */
    elf_phdr_t      *phdr;      /* Program header table if it is mapped, else NULL */
    size_t          phnum;      /* Number of program header entries */
    size_t          nseg[3];    /* corresponding entries number in program header (for text, data and tls) */

    elf_addr_t  entrypoint;    /* Entry point */

    /* 
     * Relative to dynamic digest
     */
    elf_addr_t  *got;   /* GOT table */

    const elf_reloc_t   *rel;       /* Relocation entries */
    size_t              relsize;    /* Size in bytes of relocation info */
    const elf_reloc_t   *pltrel;    /* PLT relocation entries */
    size_t              pltrelsize; /* Size in bytes of PLT relocation info */

    const elf_sym_t     *symtab;    /* Symbol table */
    const unsigned char *strtab;    /* String table */
    size_t              strsize;    /* Size in bytes of string table */

#if defined(CONFIG_CPU_MIPS)
    size_t  mips_gotsym;
    size_t  mips_local_gotno;
    size_t  mips_symtabno;
#endif

    const elf_addr_t    *buckets;   /* Hash table buckets array */
    reg_t               nbuckets;   /* Number of buckets */
    const elf_addr_t    *chains;    /* Hash table chain array */
    reg_t               nchains;    /* Number of chains */

    struct dynobj_desc_s    **dep_shobj;    /* Dependency table */
    size_t                  ndep_shobj;     /* Number of dependencies */

    void (*init)(void); /* Initialization function to call */
    void (*fini)(void); /* Termination function to call */

    /*
     * Relative to TLS
     */
    size_t  tls_modid;      /* Index of TLS if any (0 otherwise) */
    size_t  tls_max_modid;  /* Max TLS index in the dep chain (including the object) */
    size_t  tls_nb_modid;   /* Number of dependencies that uses tls (including the object) */

    /* Only for program object */
    size_t  *tls_offset_shobj;  /* offset values of the data for the program and for each dependency */
    size_t  tls_total_size;     /* total size of tls area for that program (tcb+data+dtv) */

    /* gpct pointer for double-linked list */
    /* we are forced to have dlist since link order is important */
    CONTAINER_ENTRY_TYPE(DLIST)	list_entry;
} dynobj_desc_t;

/*
 * Internal structures
 */

CONTAINER_TYPE(dynobj_list, DLIST, dynobj_desc_t, list_entry);
CONTAINER_FUNC(dynobj_list, DLIST, static inline, dynobj_list, list_entry);

struct user_dynobj_s
{
    dynobj_list_root_t prg_root;    /* List of user programs */
    dynobj_list_root_t shobj_root;  /* List of user shared objects */
} user_dynobj_list;

struct kernel_dynobj_s
{
    dynobj_list_root_t dynobj_root; /* List of kernel dynamic objects (included itself) */
} kernel_dynobj_list;

/* 
 * Private API 
 */

/* elf */
error_t
_elf_load_file(const unsigned char *pathname,
        dynobj_desc_t *dynobj);
    
/* rtld */
error_t
_rtld_load_dynobj(const unsigned char *pathname,
        dynobj_list_root_t *list_prg,
        dynobj_list_root_t *list_dep,
        dynobj_desc_t **dynobj,
        uintptr_t *threadpointer);

error_t
_rtld_lookup_sym(const elf_sym_t *ref_sym, const dynobj_desc_t *ref_dynobj,
       const elf_sym_t **def_sym, const dynobj_desc_t **def_dynobj,
       const dynobj_desc_t *root_dynobj,
       uint_fast8_t type_class);

const elf_sym_t*
_rtld_lookup_sym_dynobj(const unsigned char *name, const reg_t hash, const dynobj_desc_t *dynobj, uint_fast8_t type_class);

const dynobj_desc_t*
_rtld_lookup_ref(const dynobj_desc_t *dynobj,
        dynobj_list_root_t *list_lookup);

reg_t
_rtld_elf_hash(const unsigned char *name);

/* tls */
size_t
_rtld_tls_new_modid(void);

error_t
_rtld_tls_dynobj(dynobj_desc_t *dynobj, uintptr_t *threadpointer);

/*
 * Include CPU dependent stuff relative to rtld
 */

#include <cpu/rtld.h>

error_t
_rtld_parse_relocations(const dynobj_desc_t *dynobj, const dynobj_desc_t *root_dynobj, 
		const elf_reloc_t *rel, const size_t relsize);

#endif /* _RTLD_PRIVATE_H_ */

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

