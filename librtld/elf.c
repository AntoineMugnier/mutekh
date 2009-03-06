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
 * @param f File description of the ELF file
 * @param phdr Pointer to be set on the Program Header
 * @param phnum To be set with the number of entries in the Program Header
 * @param entry_point To be set with the entrypoint of the ELF file
 * @return error_t Error code if any
 */
static error_t 
_elf_load_headers(FILE *f, elf_phdr_t **phdr, size_t *phnum, elf_addr_t *entry_point)
{
    elf_ehdr_t *ehdr;
    uint8_t buf[ELF_HDR_SIZE];
    ssize_t nitems;

    elf_phdr_t *p;

    _rtld_debug("_elf_load_headers\n");

    /* Get the elf header */
    if ((nitems = fread(buf, sizeof(*buf), ELF_HDR_SIZE, f)) != ELF_HDR_SIZE) 
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
        if (fseek(f, ehdr->e_phoff, SEEK_SET) != 0)
            goto err_mem;

        if ((nitems = fread(p, ehdr->e_phentsize, ehdr->e_phnum, f)) != ehdr->e_phnum)
            goto err_mem;
    }

    *phdr = p;
    *phnum = ehdr->e_phnum;
    *entry_point = ehdr->e_entry;

    return 0;

err_mem:
    free(p);
err:
    return -1;
}

/* Scan Program Header and get information
 *
 * @param phdr Pointer on the Program Header
 * @param phnum Number of entries in the Program Header
 * @param dynobj Dynamic object
 * @param nseg Table to store the entries of loadable sections (text, data, etc)
 * @return error_t Error code if any
 */
static error_t 
_elf_scan_phdr(const elf_phdr_t *phdr, const size_t phnum, dynobj_desc_t *dynobj, size_t *nseg)
{
    size_t i;
    size_t nsegs = 0;

    _rtld_debug("_elf_scan_phdr\n");

    /* Scan the program header, and save key information. */
    for (i = 0; i < phnum; i++)
    {
        switch (phdr[i].p_type)
        {
            /* Program header segment */
            case PT_PHDR:
                dynobj->phdr = (const elf_phdr_t*) phdr[i].p_vaddr;
                dynobj->phsize = phdr[i].p_memsz;
                break;

            /* Loadable segments */
            case PT_LOAD:
                assert(nsegs < NB_LOAD_SEGMENTS); // text and data segments... TODO: soon TLS seg
                assert(phdr[i].p_align >= CONFIG_LIBRTLD_PAGE_SIZE);

                nseg[nsegs++] = i; /* just to remember which entries were PT_LOAD */
                break;

            /* Dynamic segment */
            case PT_DYNAMIC:
                dynobj->dynamic = (const elf_dyn_t*) phdr[i].p_vaddr;
                break;
        }
    }

    if (dynobj->dynamic == NULL)
    {
        _rtld_debug("\tloaded object is not dynamically-linked");
        return -1;
    }

    return 0;
}

/* Load loadable segments from the ELF file
 *
 * @param f File description of the ELF file
 * @param dynobj Dynamic object
 * @param phdr Pointer on the Program Header
 * @param nseg Table which contains the entries of loadable sections (text, data, etc) in the Program header
 * @return error_t Error code if any
 */
static error_t 
_elf_load_segments(FILE *f, dynobj_desc_t *dynobj, const elf_phdr_t *phdr, size_t *nseg)
{
    /* strongly assume text is segment 0 and data is segment 1 */
#define text    nseg[0]
#define data    nseg[1]

    size_t nitems;

    _rtld_debug("_elf_load_segments\n");

    dynobj->vaddrbase  = ALIGN_VALUE_LOW(phdr[text].p_vaddr, CONFIG_LIBRTLD_PAGE_SIZE);
    dynobj->mapsize = ALIGN_VALUE_UP(phdr[data].p_vaddr + phdr[data].p_memsz, CONFIG_LIBRTLD_PAGE_SIZE) - dynobj->vaddrbase;

    /*
     * Allocate sufficient contiguous pages for the object and read the object
     * into it. This will form the base for relocation.
     */
    if ((dynobj->mapbase = (uintptr_t)malloc(dynobj->mapsize)) == (uintptr_t)NULL)
    {
        _rtld_debug("\tcould not allocate %x bytes\n", dynobj->mapsize);
        return -1;
    }

    /* clear bss (at the end of data) */
    memset((void*)(dynobj->mapbase + (phdr[data].p_vaddr - phdr[text].p_vaddr) + phdr[data].p_filesz), 
                0, phdr[data].p_memsz - phdr[data].p_filesz);

    /*
     * Read the text and data segments. The BSS is already cleared.
     */
    _rtld_debug("\tload read-only segments (text, rodata, bss)\n");
    fseek(f, phdr[text].p_offset, SEEK_SET);
    if ((nitems = fread((void*)dynobj->mapbase, phdr[text].p_filesz, 1, f)) != 1) 
    {
        _rtld_debug("\t\tcould not read text segment");
        goto err_alloc;
    }

    _rtld_debug("\tload read-write segments (data)\n");
    fseek(f, phdr[data].p_offset, SEEK_SET);
    if ((nitems = fread((void*)(dynobj->mapbase + (phdr[data].p_vaddr - phdr[text].p_vaddr)), 
                        phdr[data].p_filesz, 1, f)) != 1)
    {
        _rtld_debug("\t\tcould not read data segment");
        goto err_alloc;
    }

    dynobj->relocbase = dynobj->mapbase - dynobj->vaddrbase;
    dynobj->dynamic = (elf_addr_t)dynobj->dynamic + dynobj->relocbase;

    if (dynobj->entrypoint != 0)
        dynobj->entrypoint = (elf_addr_t)dynobj->entrypoint + dynobj->relocbase;

    if (dynobj->phdr != NULL)
        dynobj->phdr = (elf_addr_t)dynobj->phdr + dynobj->relocbase;
    
    _rtld_debug("\tobject map:\n");
    _rtld_debug("\t\tmapbase: %p\n", dynobj->mapbase);
    _rtld_debug("\t\tmapsize: 0x%x\n", dynobj->mapsize);
    _rtld_debug("\t\tvaddrbase: %p\n", dynobj->vaddrbase);
    _rtld_debug("\t\trelocbase: %p\n", dynobj->relocbase);
    _rtld_debug("\t\tdynamic: %p\n", (elf_addr_t)dynobj->dynamic);
    _rtld_debug("\t\tentrypoint: %p\n", (elf_addr_t)dynobj->entrypoint);
    _rtld_debug("\t\tphdr: %p\n", (elf_addr_t)dynobj->phdr);

    return 0;

err_alloc:
    free((void*)dynobj->mapbase);
    return -1;

#undef text_seg
#undef data_seg
}

/* Process ELF headers (load and scan)
 *
 * @param f File description of the ELF file
 * @param dynobj Dynamic object
 * @param phdr Pointer to be set on the Program Header (careful: phdr will be allocated with malloc in this function)
 * @param nseg Table to store the entries of loadable sections (text, data, etc) in the Program header
 * @return error_t Error code if any
 */
static error_t
_elf_process_headers(const FILE *f, dynobj_desc_t *dynobj, elf_phdr_t **phdr, size_t nseg[NB_LOAD_SEGMENTS])
{
    elf_phdr_t *new_phdr;
    size_t phnum;
    elf_addr_t entrypoint;

    _rtld_debug("elf_process_headers\n");

    if (_elf_load_headers(f, &new_phdr, &phnum, &entrypoint) != 0)
        goto err;

    if (_elf_scan_phdr(new_phdr, phnum, dynobj, nseg) != 0)
        goto err;

    dynobj->entrypoint = entrypoint;
    dynobj->program = 1;
    *phdr = new_phdr;

    return 0;

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
_elf_load_file(const char *pathname, dynobj_desc_t *dynobj)
{
#ifdef CONFIG_RTLD_VFSNAME_HACK
    char dynobj_name[13];
    touppershortname(dynobj_name, pathname);
#else
    char *dynobj_name=pathname;
#endif

    FILE *f;

    elf_phdr_t *phdr;
    size_t nseg[NB_LOAD_SEGMENTS];

    _rtld_debug("elf_load_file\n");

    _rtld_debug("\topen file \"%s\"\n", dynobj_name);
    if ((f = fopen(dynobj_name, "r")) == NULL)
    {
        _rtld_debug("\t\tcannot open \"%s\"", dynobj_name);
        goto err_f;
    }
    dynobj->pathname = strdup(dynobj_name);

    if (_elf_process_headers(f, dynobj, &phdr, nseg) != 0)
        goto err_phdr;

    if (_elf_load_segments(f, dynobj, phdr, nseg) != 0)
        goto err_phdr;

    /* clean up */
    free(phdr);
    fclose(f);

    return 0;

err_phdr:
    free(phdr);
err_f:
    fclose(f);
    return -1;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

