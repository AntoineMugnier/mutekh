/*
    Simple Linked List template

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

#ifndef SLIST_H_
#define SLIST_H_

#include "../types.h"

#define		SLIST_TYPE_DECL(name, type)			\
struct			name##_list_s				\
{								\
  type			*next;					\
};								\
typedef type		name##_item_t;				\
typedef type	       *name##_list_t;

#define		SLIST_FUNC(name, prefix, f)			\
								\
attr name##_item_t *						\
prefix##_list_next	(name##_item_t *item)			\
{								\
  return item->f.next;						\
}								\
								\
attr void							\
prefix##_list_push	(name##_list_t *list,			\
			 name##_item_t *new)			\
{								\
  name##_list_t *l = *list;					\
								\
  item->f.next = l;						\
  *list = item;							\
}								\
								\
attr void							\
prefix##_list_pushback	(name##_list_t *list,			\
			 name##_item_t *new)			\
__attribute__ ((deprecated))					\
/* This operation is _SLOW_, double linked list would be nice */\
{								\
  while (*list)							\
    list = &(*list)->f.next;					\
								\
  prefix##_list_push(list, item);				\
}								\
								\
attr name##_list_t *						\
prefix##_list_pop	(name##_list_t	*list)			\
{								\
  name##_list_t *head = *list;					\
								\
  if (head)							\
    *list = head->f.next;					\
								\
  return head;							\
}								\
								\
attr name##_item_t *						\
prefix##_list_delete	(name##_list_t *list,			\
			 name##_item_t *item)			\
__attribute__ ((deprecated))					\
/* This operation is _SLOW_, double linked list would be nice */\
{								\
  name##_list_t *next = item->f.next;				\
								\
  while (*list != item)						\
    list = &(*list)->f.next;					\
								\
  return prefix##_list_pop(list);				\
}								\
								\
attr name##_list_t *						\
prefix##_list_popback	(name##_list_t	*list)			\
__attribute__ ((deprecated))					\
/* This operation is _SLOW_, double linked list would be nice */\
{								\
  name##_list_t *tail;						\
								\
  while (*list && (*list->next))				\
    list = &(*list)->f.next;					\
								\
  if ((tail = *list))						\
    *list = 0;							\
								\
  return tail;							\
}								\
								\
attr void							\
prefix##_list_init	(name##_list_t *list)			\
{								\
  *list = 0;							\
}								\

#define		SLIST_ROOT(name, field)		\
name##_list_t		field

#define		SLIST_ENTRY(name, field)	\
struct name##_list_s	field

#define		SLIST_FOREACH(i, init, field)	\
	for (i = init; i; i = i->field.next)

#endif

