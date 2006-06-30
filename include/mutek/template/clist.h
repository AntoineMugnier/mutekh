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

#define	__CONTAINER_CLIST_TYPE_DECL(name, type, lockname)	\
								\
struct				name##_list_s			\
{								\
  __CONT_##lockname##_FIELD	(lock);				\
  struct name##_entry_s		*head;				\
};								\
								\
struct				name##_entry_s			\
{								\
  struct name##_entry_s		*next;				\
  struct name##_entry_s		*prev;				\
};								\
								\
typedef type			name##_itembase_t;		\
typedef type *			name##_item_t;			\
typedef type *			name##_index_t;			\
typedef struct name##_list_s	name##_cont_t;			\
typedef struct name##_entry_s	name##_entry_t;


#define	__CONTAINER_CLIST_FUNC(attr, name, prefix, lockname, f)	\
								\
static inline name##_item_t					\
prefix##_get_item(name##_entry_t *entry)			\
{								\
  return (void*)(((char*)entry)					\
                 - offsetof(name##_itembase_t, f));		\
}								\
								\
attr __CONTAINER_PROTO_ISNULL(name, prefix)			\
{								\
  return !index;						\
}								\
								\
attr __CONTAINER_PROTO_GET(name, prefix)			\
{								\
  return index;							\
}								\
								\
attr __CONTAINER_PROTO_SET(name, prefix)			\
{								\
  name##_entry_t *next = item->f.next = index->f.next;		\
  name##_entry_t *prev = item->f.prev = index->f.prev;		\
								\
  __CONT_##lockname##_WRLOCK(&root->lock);			\
								\
  prev->next = &item->f;					\
  next->prev = &item->f;					\
								\
  if (root->head == &index->f)					\
    root->head = &item->f;					\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return index;							\
}								\
								\
attr __CONTAINER_PROTO_NEXT(name, prefix)			\
{								\
  return prefix##_get_item(index->f.next);			\
}								\
								\
attr __CONTAINER_PROTO_PREV(name, prefix)			\
{								\
  return prefix##_get_item(index->f.prev);			\
}								\
								\
attr __CONTAINER_PROTO_HEAD(name, prefix)			\
{								\
  name##_item_t		item;					\
								\
  __CONT_##lockname##_RDLOCK(&root->lock);			\
								\
  item = root->head ? prefix##_get_item(root->head) : 0;	\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return item;							\
}								\
								\
attr __CONTAINER_PROTO_TAIL(name, prefix)			\
{								\
  name##_item_t		item;					\
								\
  __CONT_##lockname##_RDLOCK(&root->lock);			\
								\
  item = root->head ? prefix##_get_item(root->head->prev) : 0;	\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return item;							\
}								\
								\
attr __CONTAINER_PROTO_COUNT(name, prefix)			\
{								\
  __CONT_##lockname##_RDLOCK(&root->lock);			\
								\
  name##_entry_t	*head = root->head;			\
  size_t		i = 0;					\
								\
  head = root->head;						\
								\
  if (head)							\
    do {							\
      i++;							\
      head = head->next;					\
    } while (head != root->head);				\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return i;							\
}								\
								\
attr __CONTAINER_PROTO_PUSH(name, prefix)			\
{								\
  __CONT_##lockname##_WRLOCK(&root->lock);			\
								\
  name##_entry_t	*head = root->head;			\
								\
  if (head)							\
    {								\
      name##_entry_t	*next = head;				\
      name##_entry_t	*prev = next->prev;			\
								\
      item->f.prev = prev;					\
      item->f.next = next;					\
      prev->next = &item->f;					\
      next->prev = &item->f;					\
    }								\
  else								\
    {								\
      item->f.prev = &item->f;					\
      item->f.next = &item->f;					\
    }								\
								\
  root->head = &item->f;					\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return 1; 							\
}								\
								\
attr __CONTAINER_PROTO_PUSHBACK(name, prefix)			\
{								\
  __CONT_##lockname##_WRLOCK(&root->lock);			\
								\
  name##_entry_t	*head = root->head;			\
								\
  if (head)							\
    {								\
      name##_entry_t	*next = head->prev;			\
      name##_entry_t	*prev = next->prev;			\
								\
      item->f.prev = prev;					\
      item->f.next = next;					\
      prev->next = &item->f;					\
      next->prev = &item->f;					\
    }								\
  else								\
    {								\
      item->f.prev = &item->f;					\
      item->f.next = &item->f;					\
      root->head = &item->f;					\
    }								\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return 1; 							\
}								\
								\
static inline void						\
prefix##_delete_	(name##_cont_t *root,			\
			 name##_entry_t *entry)			\
{								\
  name##_entry_t *next = entry->next;				\
  name##_entry_t *prev = entry->prev;				\
								\
  if (next == entry)						\
    root->head = 0;						\
								\
  prev->next = next;						\
  next->prev = prev;						\
}								\
								\
attr __CONTAINER_PROTO_DELETE(name, prefix)			\
{								\
  __CONT_##lockname##_WRLOCK(&root->lock);			\
								\
  if (root->head == &index->f)					\
    root->head = index->f.next;					\
								\
  prefix##_delete_(root, &index->f);				\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return index;							\
}								\
								\
attr __CONTAINER_PROTO_POP(name, prefix)			\
{								\
  __CONT_##lockname##_WRLOCK(&root->lock);			\
								\
  name##_entry_t *entry = root->head;				\
								\
  if (entry)							\
    {								\
      root->head = entry->next;					\
      prefix##_delete_(root, entry);				\
      return prefix##_get_item(entry);				\
    }								\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return 0;							\
}								\
								\
attr __CONTAINER_PROTO_POPBACK(name, prefix)			\
{								\
  __CONT_##lockname##_WRLOCK(&root->lock);			\
								\
  name##_entry_t *entry = root->head;				\
								\
  if (entry)							\
    {								\
      entry = entry->prev;					\
      prefix##_delete_(root, entry);				\
      return prefix##_get_item(entry);				\
    }								\
								\
  __CONT_##lockname##_UNLOCK(&root->lock);			\
								\
  return 0;							\
}								\
								\
attr __CONTAINER_PROTO_PUSH_ARRAY(name, prefix)			\
{								\
  uintptr_t	i;						\
								\
  /* FIXME could be optimized by writing specific code */	\
  for (i = 0; i < size; i++)					\
    prefix##_push(root, item[i]);				\
								\
  return size;							\
}								\
								\
attr __CONTAINER_PROTO_PUSHBACK_ARRAY(name, prefix)		\
{								\
  uintptr_t	i;						\
								\
  /* FIXME could be optimized by writing specific code */	\
  for (i = 0; i < size; i++)					\
    prefix##_pushback(root, item[i]);				\
								\
  return size;							\
}								\
								\
attr __CONTAINER_PROTO_POP_ARRAY(name, prefix)			\
{								\
  uintptr_t	i;						\
								\
  /* FIXME could be optimized by writing specific code */	\
  for (i = 0; i < size; i++)					\
    item[i] = prefix##_pop(root);				\
								\
  return size;							\
}								\
								\
attr __CONTAINER_PROTO_POPBACK_ARRAY(name, prefix)		\
{								\
  uintptr_t	i;						\
								\
  /* FIXME could be optimized by writing specific code */	\
  for (i = 0; i < size; i++)					\
    item[i] = prefix##_popback(root);				\
								\
  return size;							\
}								\
								\
attr __CONTAINER_PROTO_INIT(name, prefix)			\
{								\
  root->head = 0;						\
  __CONT_##lockname##_INIT(&root->lock);			\
								\
  return 0;							\
}								\
								\
attr __CONTAINER_PROTO_DESTROY(name, prefix)			\
{								\
  __CONT_##lockname##_DESTROY(&root->lock);			\
}

#endif

