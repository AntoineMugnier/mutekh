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
    Circular Double Linked List template
*/

#ifndef CLIST_H_
#define CLIST_H_

#include <stddef.h>

#include "container.h"

#define		CLIST_TYPE_DECL(name, type)			\
								\
struct				name##_entry_s			\
{								\
  struct name##_entry_s		*next;				\
  struct name##_entry_s		*prev;				\
};								\
								\
typedef type			name##_item_t;			\
typedef type *			name##_cont_t;			\
typedef struct name##_entry_s	name##_entry_t;



#define		CLIST_FUNC(attr, name, prefix, f)		\
								\
attr void							\
prefix##_init	(name##_cont_t *list)				\
{								\
  *list = 0;							\
}								\
								\
static inline name##_item_t *					\
prefix##_get_item(name##_entry_t *entry)			\
{								\
  return (void*)(((char*)entry)					\
                 - offsetof(name##_item_t, f));			\
}								\
								\
attr name##_item_t *						\
prefix##_head	(name##_cont_t *list)				\
{								\
  return *list;							\
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
prefix##_push	(name##_cont_t *list_,				\
		 name##_item_t *new)				\
{								\
  name##_item_t		*head = *list_;				\
								\
  if (head)							\
    {								\
      name##_entry_t	*prev = head->f.prev;			\
								\
      new->f.prev = prev;					\
      new->f.next = &head->f;					\
      prev->next = &new->f;					\
      head->f.prev = &new->f;					\
    }								\
  else								\
    {								\
      new->f.prev = &new->f;					\
      new->f.next = &new->f;					\
    }								\
								\
  *list_ = new;							\
}								\
								\
attr void							\
prefix##_pushback	(name##_cont_t *list_,			\
			 name##_item_t *new)			\
{								\
  name##_item_t		*list = *list_;				\
								\
  if (list)							\
    {								\
      name##_entry_t	*prev = list->f.prev;			\
								\
      new->f.next = &list->f;					\
      new->f.prev = prev;					\
      prev->next = &new->f;					\
      list->f.prev = &new->f;					\
    }								\
  else								\
    {								\
      new->f.prev = &new->f;					\
      new->f.next = &new->f;					\
      *list_ = new;						\
    }								\
}								\
								\
static inline void						\
prefix##_delete_	(name##_cont_t *list,			\
			 name##_entry_t *entry)			\
{								\
  name##_entry_t *next = entry->next;				\
  name##_entry_t *prev = entry->prev;				\
								\
  if (next == entry)						\
    *list = 0;							\
								\
  prev->next = next;						\
  next->prev = prev;						\
}								\
								\
attr name##_item_t *						\
prefix##_delete	(name##_cont_t *list,				\
		 name##_item_t *item)				\
{								\
  if (*list == item)						\
    *list = prefix##_get_item(item->f.next);			\
								\
  prefix##_delete_(list, &item->f);				\
								\
  return item;							\
}								\
								\
attr name##_item_t *						\
prefix##_pop	(name##_cont_t	*list)				\
{								\
  name##_item_t	*item = *list;					\
								\
  if (item)							\
    {								\
      *list = prefix##_get_item(item->f.next);			\
      prefix##_delete_(list, &item->f);				\
    }								\
								\
  return item;							\
}								\
								\
attr name##_item_t *						\
prefix##_popback	(name##_cont_t	*list)			\
{								\
  name##_item_t	*item = *list;					\
								\
  if (item)							\
    {								\
      item = prefix##_get_item(item->f.prev);			\
      prefix##_delete_(list, &item->f);				\
    }								\
								\
  return item;							\
}

/*
FIXME
#define		CLIST_FOREACH(prefix, i, init, f)	\
	for (i = init; i != init; i = prefix##_get_item(i->f.next))

#define		CLIST_FOREACH_REVERSE(i, init, f)	\
	for (i = init; i != init; i = prefix##_get_item(i->f.prev))
*/

#endif

