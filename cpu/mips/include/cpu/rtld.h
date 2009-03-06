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

   Based on uClibc
*/

#ifndef _MIPS_RTLD_H_
#define _MIPS_RTLD_H_

#define elf_machine_type_class(type)    ELF_RTYPE_CLASS_PLT
/* MIPS does not have COPY relocs */
#define ELF_NO_COPY_RELOCS

#define INIT_GOT(MODULE)                                                \
    do {                                                                \
        uint32_t idx;  													\
                                                                        \
        /* Fill in first two GOT entries according to the ABI */        \
        /* MODULE->got[0] = (Elf_Addr) _rtld_bind_start;*/              \
        /* MODULE->got[1] = (Elf_Addr) MODULE;             */           \
                                                                        \
        /* Add load address displacement to all local GOT entries */    \
        idx = 2;                                                        \
        while (idx < MODULE->mips_local_gotno)                          \
            MODULE->got[idx++] += (elf_addr_t) MODULE->relocbase;       \
                                                                        \
    } while (0)

/* special for mips */
error_t
_rtld_mips_global_got_relocations (const dynobj_desc_t *dynobj, const dynobj_desc_t *root_dynobj);

#endif //_MIPS_RTLD_H_

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

