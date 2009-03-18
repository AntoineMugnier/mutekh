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

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include <hexo/endian.h>

#include <rtld/rtld-private.h>

#define ELF_HDR_SIZE 128 /* based on linux, the header should be less than 128bytes */

/* Load ELF header, perform test and load Program Header
 *
 * @param file File descriptor on the ELF file
 * @param dynobj Dynamic object
 * @return error_t Error code if any
 */
static error_t 
_elf_load_headers(FILE *file, dynobj_desc_t *dynobj)
{
    elf_ehdr_t *ehdr;
    uint8_t buf[ELF_HDR_SIZE];

    elf_phdr_t *p;

    _rtld_debug("_elf_load_headers\n");

    /* Get the elf header */
    if (fread(buf, sizeof(*buf), ELF_HDR_SIZE, file) != ELF_HDR_SIZE) 
    {
        _rtld_debug("\tError while reading elf file\n");
        goto err;
    }

    ehdr = (elf_ehdr_t*)buf;

    /* Make sure the file is valid */
    if (ehdr->e_ident[EI_MAG0] != ELFMAG0
            || ehdr->e_ident[EI_MAG1] != ELFMAG1
            || ehdr->e_ident[EI_MAG2] != ELFMAG2
            || ehdr->e_ident[EI_MAG3] != ELFMAG3) 
    {
        _rtld_debug("\tinvalid file format\n");
        goto err;
    }
    if (ehdr->e_ident[EI_CLASS] != ELF_TARG_CLASS || ehdr->e_ident[EI_DATA] != ELF_TARG_DATA)
    {
        _rtld_debug("\tunsupported file layout\n");
        goto err;
    }
    if (ehdr->e_ident[EI_VERSION] != EV_CURRENT || ehdr->e_version != EV_CURRENT)
    {
        _rtld_debug("\tunsupported file version\n");
        goto err;
    }
    if (ehdr->e_type != ET_EXEC && ehdr->e_type != ET_DYN)
    {
        _rtld_debug("\tunsupported file type\n");
        goto err;
    }
    if (ehdr->e_machine != ELF_TARG_ARCH)
    {
        _rtld_debug("\tunsupported machine\n");
        goto err;
    }

    /* Get the program headers */
    assert(ehdr->e_phentsize == sizeof(elf_phdr_t));
    if ((p = malloc(ehdr->e_phnum*ehdr->e_phentsize)) == NULL)
        goto err;
    

    /* If it's contained in the previous read, let's memcopy it */
    if (ehdr->e_phoff + ehdr->e_phnum*ehdr->e_phentsize <= ELF_HDR_SIZE)
        memcpy(p, buf + ehdr->e_phoff, ehdr->e_phnum*ehdr->e_phentsize);

    /* Otherwise, let's read it from the file */
    else
    {
        if (fseek(file, ehdr->e_phoff, SEEK_SET) != 0)
            goto err_mem;

        if (fread(p, ehdr->e_phentsize, ehdr->e_phnum, file) != ehdr->e_phnum)
            goto err_mem;
    }

    /* we could retrieve phdr later via the text segment, but for DSO, 
     * we don't have the information directly, so let's make that 
     * generic: we keep the allocated phdr and ignore the phdr in text segment */
    dynobj->phdr = p;
    dynobj->phnum = ehdr->e_phnum;
    dynobj->entrypoint = ehdr->e_entry;

    return 0;

err_mem:
    free(p);
err:
    return -1;
}

/* Scan Program Header and get information
 *
 * @param dynobj Dynamic object
 * @return error_t Error code if any
 */
#if defined(CONFIG_LIBRTLD_DEBUG)
static const char* const _rtld_phdr_type_names[8] = {
    "PT_NULL", "PT_LOAD", "PT_DYNAMIC", "PT_INTERP",
    "PT_NOTE", "PT_SHLIB", "PT_PHDR", "PT_TLS"
};
#endif
static error_t 
_elf_scan_phdr(dynobj_desc_t *dynobj)
{
    size_t i;
    size_t nsegs = 0;

    _rtld_debug("_elf_scan_phdr\n");

    /* Scan the program header, and save key information. */
    for (i = 0; i < dynobj->phnum; i++)
    {
        elf_phdr_t *phdr_entry = &dynobj->phdr[i];

        if (phdr_entry->p_type < 8)
            _rtld_debug("\ttype \'%s\'\n", _rtld_phdr_type_names[phdr_entry->p_type]);
        else
            _rtld_debug("\ttype \'0x%x\'\n", phdr_entry->p_type);
        switch (phdr_entry->p_type)
        {
            /* Program header segment */
            case PT_PHDR:
                /* let's skip it and keep our mallocated phdr */
                /* too bad for memory consumption */
                break;

            case PT_INTERP:
                /* It's the only way we can know it's a program and not a shared library that we are loading */
                dynobj->program = 1;
                break;

            /* Loadable segments */
            case PT_LOAD:
                assert(nsegs < 2); // text and data segments
                assert(phdr_entry->p_align >= CONFIG_LIBRTLD_PAGE_SIZE);

                dynobj->nseg[nsegs++] = i; /* just to remember which entries were PT_LOAD */
                break;

            /* TLS segment */
            case PT_TLS:
                dynobj->nseg[RTLD_TLS_SEG] = i; /* just to remember which entries was PT_TLS */
                if (dynobj->program)
                    /* if tls is used, program always get the modid #1 */
                    dynobj->tls_modid = 1;
                else
                    /* otherwise, we get a new modid */
                    dynobj->tls_modid = _tls_get_new_modid();
                dynobj->tls_nb_modid = 1;
                dynobj->tls_max_modid = dynobj->tls_modid;
                _rtld_debug("\t\tmodid = %d\n", dynobj->tls_modid);
                break;

            /* Dynamic segment */
            case PT_DYNAMIC:
                dynobj->dynamic = (const elf_dyn_t*) phdr_entry->p_vaddr;
                break;

            default:
                _rtld_debug("\t\tignored entry\n");
                break;
        }
    }

    if (dynobj->dynamic == NULL)
    {
        _rtld_debug("\terror: loaded object is not dynamically-linked");
        return -1;
    }

    return 0;
}

/* Load loadable segments from the ELF file
 *
 * @param file File descriptor on the ELF file
 * @param dynobj Dynamic object
 * @return error_t Error code if any
 */
static error_t 
_elf_load_segments(FILE *file, dynobj_desc_t *dynobj)
{
#define text_seg    dynobj->phdr[dynobj->nseg[RTLD_TEXT_SEG]]
#define data_seg    dynobj->phdr[dynobj->nseg[RTLD_DATA_SEG]]

    _rtld_debug("_elf_load_segments\n");

    dynobj->vaddrbase  = ALIGN_VALUE_LOW(text_seg.p_vaddr, CONFIG_LIBRTLD_PAGE_SIZE);
    dynobj->mapsize = ALIGN_VALUE_UP(data_seg.p_vaddr + data_seg.p_memsz,
            CONFIG_LIBRTLD_PAGE_SIZE) - dynobj->vaddrbase;

    /*
     * Allocate sufficient contiguous pages for the object and read the object
     * into it. This will form the base for relocation.
     */
    if ((dynobj->mapbase = (uintptr_t)malloc(dynobj->mapsize)) == (uintptr_t)NULL)
    {
        _rtld_debug("\tcould not allocate %x bytes\n", dynobj->mapsize);
        return -1;
    }

    /*
     * Load the text segment in memory
     */
    _rtld_debug("\tload read-only segments (text, rodata, bss)\n");
    fseek(file, text_seg.p_offset, SEEK_SET);
    if (fread((void*)dynobj->mapbase, text_seg.p_filesz, 1, file) != 1) 
    {
        _rtld_debug("\t\tcould not read text segment");
        goto err_alloc;
    }

    /*
     * Load the data segment in memory
     */
    _rtld_debug("\tload read-write segments (data)\n");
    fseek(file, data_seg.p_offset, SEEK_SET);
    if (fread((void*)(dynobj->mapbase + (data_seg.p_vaddr - text_seg.p_vaddr)), 
                        data_seg.p_filesz, 1, file) != 1)
    {
        _rtld_debug("\t\tcould not read data segment");
        goto err_alloc;
    }

    /* clear bss (at the end of data) */
    memset((void*)(dynobj->mapbase + (data_seg.p_vaddr - text_seg.p_vaddr) + data_seg.p_filesz), 
                0, data_seg.p_memsz - data_seg.p_filesz);

    /* relocation value */
    dynobj->relocbase = dynobj->mapbase - dynobj->vaddrbase;

    /* relocate some stuffs */
    dynobj->dynamic = (elf_dyn_t*)((elf_addr_t)dynobj->dynamic + dynobj->relocbase);
    if (dynobj->entrypoint != 0)
        dynobj->entrypoint = dynobj->entrypoint + dynobj->relocbase;

    _rtld_debug("\tobject map:\n");
    _rtld_debug("\t\tmapbase: %p\n",    (void*)dynobj->mapbase);
    _rtld_debug("\t\tmapsize: 0x%x\n",  dynobj->mapsize);
    _rtld_debug("\t\tvaddrbase: %p\n",  (void*)dynobj->vaddrbase);
    _rtld_debug("\t\trelocbase: %p\n",  (void*)dynobj->relocbase);
    _rtld_debug("\t\tdynamic: %p\n",    (void*)dynobj->dynamic);
    _rtld_debug("\t\tentrypoint: %p\n", (void*)dynobj->entrypoint);
    _rtld_debug("\t\tphdr: %p\n",       (void*)dynobj->phdr);

    return 0;

err_alloc:
    free((void*)dynobj->mapbase);
    return -1;

#undef text_seg
#undef data_seg
}

/* Process ELF headers (load and scan)
 *
 * @param file File descriptor on the ELF file
 * @param dynobj Dynamic object
 * @return error_t Error code if any
 */
static error_t
_elf_process_headers(FILE *file, dynobj_desc_t *dynobj)
{
    _rtld_debug("elf_process_headers\n");

    if (_elf_load_headers(file, dynobj) != 0)
        goto err;

    if (_elf_scan_phdr(dynobj) != 0)
        goto err_mem;

    return 0;

err_mem:
    free(dynobj->phdr);
err:
    return -1;
}

/* Load ELF file into memory
 *
 * @param pathname Pathname of the ELF file
 * @param dynobj Dynamic object
 * @return error_t Error code if any
 */
error_t
_elf_load_file(const unsigned char *pathname, dynobj_desc_t *dynobj)
{
#ifdef CONFIG_RTLD_VFSNAME_HACK
    char dynobj_name[13];
    touppershortname(dynobj_name, pathname);
#else
    char *dynobj_name=pathname;
#endif

    FILE* file;

    _rtld_debug("elf_load_file\n");

    _rtld_debug("\topen file \"%s\"\n", dynobj_name);
    if ((file = fopen(dynobj_name, "r")) == NULL)
    {
        _rtld_debug("\t\tcannot open \"%s\"", dynobj_name);
        goto err_f;
    }
    dynobj->pathname = (unsigned char*)strdup(dynobj_name);

    if (_elf_process_headers(file, dynobj) != 0)
        goto err_f;

    if (_elf_load_segments(file, dynobj) != 0)
        goto err_phdr;

    fclose(file);

    return 0;

err_phdr:
    free(dynobj->phdr);
err_f:
    fclose(file);
    return -1;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

