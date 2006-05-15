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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef CONTAINER_H_
#define CONTAINER_H_

#define		CONTAINER_PROTOTYPE(attr, name, prefix)		\
attr void							\
prefix##_init	(name##_cont_t *cont);				\
								\
attr void							\
prefix##_push	(name##_cont_t *cont,				\
								\
attr name##_item_t *						\
prefix##_delete	(name##_cont_t *cont,				\
		 name##_item_t *item);				\
								\
attr name##_item_t *						\
prefix##_pop	(name##_cont_t	*cont);				\
								\
attr name##_item_t *						\
prefix##_head	(name##_cont_t *list);				\
								\
attr name##_item_t *						\
prefix##_next	(name##_item_t *item);				\
								\
attr name##_item_t *						\
prefix##_prev	(name##_item_t *item);				\
								\
attr void							\
prefix##_pushback	(name##_cont_t *cont,			\
			 name##_item_t *new);			\
								\
attr name##_item_t *						\
prefix##_popback	(name##_cont_t	*cont);			\

#endif

