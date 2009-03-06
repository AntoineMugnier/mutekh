/* mips/mipsel ELF shared library loader suppport
 *
 * Copyright (C) 2002, Steven J. Hill (sjhill@realitydiluted.com)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. The name of the above contributors may not be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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

   Based on uClibc
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtld/rtld-private.h>

/* Relocate global GOT of a given mips object
 *
 * @param dynobj Object to relocate
 * @param root_dynobj Object which is the root of the relocation chain
 * @return error_t Error code if any
 */
error_t
_rtld_mips_global_got_relocations (const dynobj_desc_t *dynobj, const dynobj_desc_t *root_dynobj)
{
	size_t sym_idx;
	elf_addr_t *got_entry;

	_rtld_debug("_rtld_mips_global_got_relocations\n");

	/* goto global entries, skip local entries */
	got_entry = dynobj->got + dynobj->mips_local_gotno;

	/* loop through all the mapped symbol */
	for (sym_idx = dynobj->mips_gotsym; sym_idx < dynobj->mips_symtabno; sym_idx++, got_entry++)
	{
		const elf_sym_t *ref_sym;
		const elf_sym_t *def_sym;
		const dynobj_desc_t *def_dynobj;

		ref_sym = dynobj->symtab + sym_idx;

		_rtld_debug("\trelocate global symbol \"%s\"\n", dynobj->strtab + ref_sym->st_name);

		if (ref_sym->st_shndx == SHN_UNDEF)
		{
			{
				if (_rtld_lookup_sym(ref_sym, dynobj, &def_sym, &def_dynobj, root_dynobj, ELF_RTYPE_CLASS_PLT) != 0)
					return -1;

				_rtld_debug("\trelocate symbol \"%s\": ", dynobj->strtab + ref_sym->st_name);
                _rtld_debug("from @%p", *got_entry);
				*got_entry = def_sym->st_value + def_dynobj->relocbase;
                _rtld_debug(" to @%p\n", *got_entry);
			}
		}
		else if (ref_sym->st_shndx == SHN_COMMON)
        {
			if (_rtld_lookup_sym(ref_sym, dynobj, &def_sym, &def_dynobj, root_dynobj, ELF_RTYPE_CLASS_PLT) != 0)
				return -1;

			_rtld_debug("\trelocate symbol \"%s\": ", dynobj->strtab + ref_sym->st_name);
			_rtld_debug("from @%p", *got_entry);
			*got_entry = def_sym->st_value + def_dynobj->relocbase;
			_rtld_debug(" to @%p\n", *got_entry);
		}
		else if (ELF_ST_TYPE(ref_sym->st_info) == STT_SECTION)
        {
			if (ref_sym->st_other == 0)
            {
				_rtld_debug("\trelocate symbol \"%s\": ", dynobj->strtab + ref_sym->st_name);
                _rtld_debug("from @%p to @%p\n", *got_entry, *got_entry + dynobj->relocbase);
				*got_entry += dynobj->relocbase;
            }
		}
        else {
			if (_rtld_lookup_sym(ref_sym, dynobj, &def_sym, &def_dynobj, root_dynobj, ELF_RTYPE_CLASS_PLT) != 0)
				return -1;

			_rtld_debug("\trelocate symbol \"%s\": ", dynobj->strtab + ref_sym->st_name);
			_rtld_debug("from @%p", *got_entry);
			*got_entry = def_sym->st_value + def_dynobj->relocbase;
			_rtld_debug(" to @%p\n", *got_entry);
		}
	}

    return 0;
}

/* Perform the relocations in the given object
 *
 * @param dynobj Object to relocate
 * @param root_dynobj Object which is the root of the relocation chain
 * @param rel Table of relocation entries
 * @param relsize Size of the table of relocation entries
 * @return error_t Error code if any
 */
error_t
_rtld_parse_relocations(const dynobj_desc_t *dynobj, const dynobj_desc_t *root_dynobj, 
		const elf_reloc_t *rel, const size_t relsize)
{
	const elf_reloc_t *rel_entry;
	const elf_reloc_t *rel_end;
    
    _rtld_debug("_rtld_parse_relocations\n");

    /* relsize is in bytes */
	rel_end = (const elf_reloc_t*)((const char*)rel + relsize);

	for (rel_entry = rel; rel_entry < rel_end; rel_entry++)
	{
        elf_addr_t *where;
        reg_t reloc_type;

        size_t sym_index;
		const elf_sym_t *sym;

        sym_index = ELF_R_SYM(rel_entry->r_info);
        sym = dynobj->symtab + sym_index;

		_rtld_debug("\trelocate symbol \"%s\": ", dynobj->strtab + sym->st_name);

		where = (elf_addr_t*) (dynobj->relocbase + rel_entry->r_offset);
        reloc_type = ELF_R_TYPE(rel_entry->r_info);

		switch (reloc_type)
        {
            case R_MIPS_NONE:
                _rtld_debug("(R_MIPS_NONE)\n");
                break;

            case R_MIPS_REL32:
                _rtld_debug("(R_MIPS_REL32) from @%p", *where);
                if (sym_index)
                {
                    if (sym_index < dynobj->mips_gotsym)
                        *where += sym->st_value + dynobj->relocbase;
                    else
                        *where += dynobj->got[sym_index + dynobj->mips_local_gotno - dynobj->mips_gotsym];
                } else
                    *where += dynobj->relocbase;
                _rtld_debug(" to @%p\n", *where);
                break;

            default:
                _rtld_debug("\tunsupported relocation type %d\n", reloc_type);
                return -1;
        }
    }

    return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

