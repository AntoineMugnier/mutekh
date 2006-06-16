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

/*
    Double Linked List template.

    The list is implemented with head and tail pointers in container
    root. Null pointers are used as list terminator.
*/

#ifndef DLIST_H_
#define DLIST_H_

#include <stddef.h>

#include "container.h"

#define		DLIST_TYPE_DECL(name, type)			\
								\
struct				name##_list_s			\
{								\
  struct name##_entry_s		*head;				\
  struct name##_entry_s		*tail;				\
};								\
								\
struct				name##_entry_s			\
{								\
  struct name##_entry_s		*next;				\
  struct name##_entry_s		*prev;				\
};								\
								\
typedef type			name##_item_t;			\
typedef struct name##_list_s	name##_cont_t;			\
typedef struct name##_entry_s	name##_entry_t;



#define		DLIST_FUNC(attr, name, prefix, f)		\
								\
static inline name##_item_t *					\
prefix##_get_item(name##_entry_t *entry)			\
{								\
  return (void*)(((char*)entry)					\
                 - offsetof(name##_item_t, f));			\
}								\
								\
attr name##_item_t *						\
prefix##_next	(name##_item_t *item)				\
{								\
  return prefix##_get_item(item->f.next);			\
}								\
								\
attr name##_item_t *						\
prefix##_prev	(name##_item_t *item)				\
{								\
  return prefix##_get_item(item->f.prev);			\
}								\
								\
attr void							\
prefix##_push	(name##_cont_t *list,				\
		 name##_item_t *new)				\
{								\
  name##_entry_t	*head = list->head;			\
								\
  new->f.prev = 0;						\
  new->f.next = head;						\
  *(head ? &head->prev : &list->tail) = &new->f;		\
  list->head = &new->f;						\
}								\
								\
attr void							\
prefix##_pushback	(name##_cont_t *list,			\
			 name##_item_t *new)			\
{								\
  name##_entry_t	*tail = list->tail;			\
								\
  new->f.next = 0;						\
  new->f.prev = tail;						\
  *(tail ? &tail->next : &list->head) = &new->f;		\
  list->tail = &new->f;						\
}								\
								\
static inline void						\
prefix##_delete_	(name##_cont_t *list,			\
			 name##_entry_t *entry)			\
{								\
  name##_entry_t *next = entry->next;				\
  name##_entry_t *prev = entry->prev;				\
								\
  *(prev ? &prev->next : &list->head) = next;			\
  *(next ? &next->prev : &list->tail) = prev;			\
}								\
								\
attr name##_item_t *						\
prefix##_delete	(name##_cont_t *list,				\
		 name##_item_t *item)				\
{								\
  prefix##_delete_(list, &item->f);				\
								\
  return item;							\
}								\
								\
attr name##_item_t *						\
prefix##_pop	(name##_cont_t	*list)				\
{								\
  name##_item_t	*item = prefix##_get_item(list->head);		\
								\
  prefix##_delete_(list, list->head);				\
								\
  return item;							\
}								\
								\
attr name##_item_t *						\
prefix##_popback	(name##_cont_t	*list)			\
{								\
  name##_item_t	*item = prefix##_get_item(list->tail);		\
								\
  prefix##_delete_(list, list->tail);				\
								\
  return item;							\
}								\
								\
attr void							\
prefix##_init	(name##_cont_t *list)				\
{								\
  list->head = 0;						\
  list->tail = 0;						\
}

#define		DLIST_FOREACH(i, list, f)	\
	for (i = prefix##_get_item(list->head); i; i = i->f.next)

#define		DLIST_FOREACH_REVERSE(i, list, f)	\
	for (i = prefix##_get_item(list->tail); i; i = i->f.prev)

#endif

