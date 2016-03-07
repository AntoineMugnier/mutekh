/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <device/class/dma.h>

extern inline error_t dev_dma_spin_copy(const struct device_dma_s *accessor,
                                        struct dev_dma_rq_s* rq);

# ifdef CONFIG_MUTEK_CONTEXT_SCHED
extern inline error_t dev_dma_wait_copy(const struct device_dma_s *accessor,
                                        struct dev_dma_rq_s* rq);

# endif

