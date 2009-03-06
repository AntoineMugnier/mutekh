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
    
    Based on glibc
*/
#ifndef _RTLD_TYPES_H_
#define _RTLD_TYPES_H_

#include <rtld/elf.h>
#include <cpu/elf.h>

/*
 * 32 or 64 bits target CPU
 */
#if ELF_TARG_CLASS == ELFCLASS32

extern Elf32_Dyn _DYNAMIC [];

typedef Elf32_Ehdr	elf_ehdr_t;
typedef Elf32_Phdr	elf_phdr_t;

typedef Elf32_Dyn	elf_dyn_t;
typedef Elf32_Sym	elf_sym_t;
typedef Elf32_Rel	elf_rel_t;
typedef Elf32_Rela	elf_rela_t;

typedef Elf32_Addr	elf_addr_t;
typedef Elf32_Word	elf_word_t;

#define ELF_ST_BIND ELF32_ST_BIND
#define ELF_ST_TYPE ELF32_ST_TYPE
#define ELF_ST_INFO ELF32_ST_INFO

#define ELF_R_SYM  ELF32_R_SYM
#define ELF_R_TYPE ELF32_R_TYPE
#define ELF_R_INFO ELF32_R_INFO

#elif ELF_TARG_CLASS == ELFCLASS64

extern Elf64_Dyn _DYNAMIC [];

typedef Elf64_Ehdr	elf_ehdr_t;
typedef Elf64_Phdr	elf_phdr_t;

typedef Elf64_Dyn	elf_dyn_t;
typedef Elf64_Sym	elf_sym_t;
typedef Elf64_Rel	elf_rel_t;
typedef Elf64_Rela	elf_rela_t;

typedef Elf64_Addr	elf_addr_t;
typedef Elf64_Word	elf_word_t;

#define ELF_ST_BIND ELF64_ST_BIND
#define ELF_ST_TYPE ELF64_ST_TYPE
#define ELF_ST_INFO ELF64_ST_INFO

#define ELF_R_SYM  ELF64_R_SYM
#define ELF_R_TYPE ELF64_R_TYPE
#define ELF_R_INFO ELF64_R_INFO

#else

#error "No ELF_TARG_CLASS defined for your CPU"

#endif

/*
 * two types of relocation
 */
#ifdef ELF_USE_RELOCA
typedef elf_rela_t elf_reloc_t;
# define DT_SUPPORTED_RELOC_TYPE	DT_RELA
#else
typedef elf_rel_t elf_reloc_t;
# define DT_SUPPORTED_RELOC_TYPE	DT_REL
#endif

/* Reloc type classes as returned by elf_machine_type_class().
   ELF_RTYPE_CLASS_PLT means this reloc should not be satisfied by
   some PLT symbol, ELF_RTYPE_CLASS_COPY means this reloc should not be
   satisfied by any symbol in the executable.  Some architectures do
   not support copy relocations.  In this case we define the macro to
   zero so that the code for handling them gets automatically optimized
   out.  */
#ifdef ELF_NO_COPY_RELOCS
# define ELF_RTYPE_CLASS_COPY   (0x0)
#else
# define ELF_RTYPE_CLASS_COPY   (0x2)
#endif
#define ELF_RTYPE_CLASS_PLT (0x1)

#endif /* _RTLD_TYPES_H_ */

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4


