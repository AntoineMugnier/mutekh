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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#if !defined(CONTEXT_H_) || defined(CPU_CONTEXT_H_)
#error This file can not be included directly
#else

extern void	*__softregs_start;
extern void	*__softregs_end;

struct		cpu_context_s
{
};

static inline void
cpu_context_switch(struct context_s *old, struct context_s *new)
{
  asm ("bgnd"); /* XXX */

#if 0
  asm (/* save PC */
       "	jsr	1f				\n"
       "1:						\n"
       /* push CCR */
       "	pshc					\n"
       /* switch stack */
       "	sts	[0,x]				\n"
       "	lds	[0,y]				\n"
       /* pop CCR */
       "	pulc					\n"
       /* restore PC */
       "	rts					\n"
       : "=x" (old->stack_ptr)
       : "y" (new->stack_ptr)
       : "a", "b", "memory");

#endif
  /* restore tls */
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_jumpto(struct context_s *new)
{
  asm ("bgnd"); /* XXX */
}

static inline void
__attribute__((always_inline, noreturn))
cpu_context_set_stack(uintptr_t stack, void *jumpto)
{
  asm ("bgnd"); /* XXX */
}

#endif

