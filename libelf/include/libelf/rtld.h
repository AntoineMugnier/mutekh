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

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_dlist.h>

#include <libelf/elf.h>
#include <libelf/tls.h>

/*
 * RTLD object
 */
struct dynobj_rtld_s
{
    /*
     * Elf
     */
    struct obj_elf_s elf;

    /*
     * Misc
     */
    uint_fast32_t   refcount;   /* number of uses of this object (FIXME: memory management in all rtld is total bullshit) */
    bool_t          symbolic;   /* True if generated with "-Bsymbolic" */
    bool_t          relocated;  /* True if the object is finished being loaded */

    /*
     * Relative to dynamic digest
     */
    elf_addr_t  *got;   /* GOT table */

    const elf_reloc_t   *rel;       /* Relocation entries */
    size_t              relsize;    /* Size in bytes of relocation info */
    const elf_reloc_t   *pltrel;    /* PLT relocation entries */
    size_t              pltrelsize; /* Size in bytes of PLT relocation info */

    const elf_sym_t     *symtab;    /* Symbol table */
    const char          *strtab;    /* String table */
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

    struct dynobj_rtld_s    **dep_shobj;    /* Dependency table */
    size_t                  ndep_shobj;     /* Number of dependencies */

    void (*init)(void); /* Initialization function to call */
    void (*fini)(void); /* Termination function to call */

#if defined(CONFIG_LIBELF_RTLD_TLS)
    struct dynobj_tls_s tls;
#endif

    /* gpct pointer for double-linked list */
    /* we are forced to have dlist since link order is important */
    CONTAINER_ENTRY_TYPE(DLIST)	list_entry;
};

/* List type for rtld objects */
CONTAINER_TYPE(dynobj_list, DLIST, struct dynobj_rtld_s, list_entry);
CONTAINER_FUNC(dynobj_list, DLIST, static inline, dynobj_list, list_entry);

/* 
 * Scanning callback
 */
#define LIBELF_SCAN_SEGS(n) error_t (n) (uintptr_t base, size_t size, void *priv_data)
typedef LIBELF_SCAN_SEGS(libelf_scan_segments_t);

struct segs_scan_ctxt_s
{
    libelf_scan_segments_t *fcn_scan_segs;
    void *priv_data;
};
extern struct segs_scan_ctxt_s segs_scan_ctxt;


/* 
 * Functions prototypes
 */

error_t rtld_init (void);

error_t rtld_configure_callbacks (libelf_alloc_segments_t *alloc_segs,
        libelf_alloc_tls_t *alloc_tls,
        libelf_scan_segments_t *scan_segs,
        void *priv_data);

error_t rtld_open (struct dynobj_rtld_s **dynobj, const char *pathname);

error_t rtld_sym (const struct dynobj_rtld_s *dynobj, const char *name, uintptr_t *sym);

error_t rtld_tls (const struct dynobj_rtld_s *dynobj, uintptr_t *tls, uintptr_t *threadpointer);

error_t rtld_close (const struct dynobj_rtld_s *dynobj);


/*
 * cpu dependent stuff
 */
#include <cpu/rtld.h>
void _rtld_init_got (const struct dynobj_rtld_s *dynobj);
error_t _rtld_parse_nonplt_relocations(const struct dynobj_rtld_s *dynobj, const struct dynobj_rtld_s *root_dynobj);
error_t _rtld_parse_plt_relocations(const struct dynobj_rtld_s *dynobj, const struct dynobj_rtld_s *root_dynobj);

#endif /* _RTLD_H_ */

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

