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

#ifndef __CONTAINER_H_
#define __CONTAINER_H_

#include <hexo/error.h>
#include <hexo/types.h>

/*
  ===============
  CONTAINER TYPES
  ===============

  The container types must be defined before field or variable
  decalrations. The CONTAINER_TYPE_DECL macro must be used to define
  all types associated with a container.

  CONTAINER_TYPE_DECL(name, algorithm, type, lockname, ...)

	* `name' is the new container name defined by this macro

	* `algorithm' is the container algorithm identifier (CLIST, DLIST,
	  RING, VECTOR ...)

	* `type' is the type of the item handled by the container

	* `...' specific parameters required by the container algorithm

	* `lockname' is the locking policy/algorithm used (NOLOCK, SPIN,
	  SPIN_IRQ, PTHREAD_MUTEX, ...)

  name##_itembase_t

	name##_itembase_t is the item type as passed to the
	CONTAINER_TYPE_DECL macro.

  name##_item_t

	is the item type, same as name##_itembase_t or pointer to
	name##_itembase_t depending on algorithm used.

  name##_index_t

	is either an integer index type or a pointer to
	name##_itembase_t depending on algorithm used.

  name##_cont_t

	name##_cont_t is the type of the root container object. It can
	be an array, a pointer to name##_itembase_t or a mode complex
	structure.

  name##_entry_t

	A field of type name##_entry_t must be added in user item
	structure for container algorithms using linked lists.

  ===================
  CONTAINER FUNCTIONS
  ===================

  Functions used to access the container are defined using the
  CONTAINER_FUNC macro.

  CONTAINER_FUNC(qualifier, name, algorithm, prefix, lockname, ...)

	* `qualifier' functions qualifier (ex: static inline)

	* `name' is the container name. Must be the same as used for
	  the container types declaration.

	* `algorithm' is the container algorithm identifier (CLIST,
	  DLIST, RING, VECTOR ...). Must be the same as used for the
	  container types declaration.

	* `prefix' is the function name prefix used for functions
  	  decalaration. (ex: `foo' will define functions named
  	  foo_get, foo_push, ... )

	* `...' specific parameters required by the container
	  algorithm. This may include linked list field name in item
	  structure, value test field name in item structure and
	  others.

	* `lockname' is the locking policy/algorithm used (NOLOCK, SPIN,
	  SPIN_IRQ, PTHREAD_MUTEX, ...) to access container

 */

/***********************************************************************
 *	define all container functions prototypes
 */

/**
   Check index validity

   @param index index value to check
   @return non zero if index is a null index
*/
#define __CONTAINER_PROTO_ISNULL(name, prefix)			\
__bool_t							\
prefix##_isnull	(name##_index_t index)


/**
   Get the item at the specified index.

   @param root container root
   @param index index of the item
   @return item
 */
#define __CONTAINER_PROTO_GET(name, prefix)			\
name##_item_t							\
prefix##_get	(name##_cont_t *root, name##_index_t index)


/**
   Set an item and replace the old item at the specified index.

   @param root container root
   @param index index of the item to replace
   @return old replaced item
 */
#define __CONTAINER_PROTO_SET(name, prefix)			\
void								\
prefix##_set	(name##_cont_t *root,				\
		 name##_index_t index,				\
		 name##_item_t item)


/**
   Return the index of the next item

   @param root container root
   @param index index of the current item
   @return index to the next item
 */
#define __CONTAINER_PROTO_NEXT(name, prefix)			\
name##_index_t							\
prefix##_next	(name##_cont_t *root, name##_index_t index)


/**
   Return the index of the previous item

   @param root container root
   @param index index of the current item
   @return index to the previous item
 */
#define __CONTAINER_PROTO_PREV(name, prefix)			\
name##_index_t							\
prefix##_prev	(name##_cont_t *root, name##_index_t index)


/**
   Return the index of the first item in the container

   @param root container root
   @return index to the first item
 */
#define __CONTAINER_PROTO_HEAD(name, prefix)			\
name##_index_t							\
prefix##_head	(name##_cont_t *root)


/**
   Return the index of the last item in the container

   @param root container root
   @return index to the last item
 */
#define __CONTAINER_PROTO_TAIL(name, prefix)			\
name##_index_t							\
prefix##_tail	(name##_cont_t *root)


/**
   Return the current items count in the container.

   @param root container root
   @return items count
 */
#define __CONTAINER_PROTO_COUNT(name, prefix)			\
size_t								\
prefix##_count	(name##_cont_t *root)


/**
   Return the maximum items count in the container.

   @param root container root
   @return max items count
 */
#define __CONTAINER_PROTO_MAXCOUNT(name, prefix)		\
size_t								\
prefix##_maxcount(name##_cont_t *root)


/**
   Delete the item at the given index in the container

   @param root container root
   @param index index of the item to delete
   @return deleted item
 */
#define __CONTAINER_PROTO_DELETE(name, prefix)			\
void								\
prefix##_delete	(name##_cont_t *root, name##_index_t index)


/**
   Insert an item before the container first item. The function return
   pushed items count which can be lower than expected if the
   container is full (return 1 or 0).

   @param root container root
   @param item item to insert
 */
#define __CONTAINER_PROTO_PUSH(name, prefix)			\
size_t								\
prefix##_push	(name##_cont_t *root, name##_item_t item)


/**
   Insert an item after the container last item. The function return
   pushed items count which can be lower than expected if the
   container is full (return 1 or 0).

   @param root container root
   @param item item to insert
 */
#define __CONTAINER_PROTO_PUSHBACK(name, prefix)		\
size_t								\
prefix##_pushback(name##_cont_t *root, name##_item_t item)


/**
   Remove and get the container first item.  0 or NULL is returned if
   the container is empty.

   @param root container root
   @return removed item
 */
#define __CONTAINER_PROTO_POP(name, prefix)			\
name##_item_t							\
prefix##_pop	(name##_cont_t	*root)


/**
   Remove and get the container last item. 0 or NULL is returned if
   the container is empty.

   @param root container root
   @return removed item
 */
#define __CONTAINER_PROTO_POPBACK(name, prefix)			\
name##_item_t							\
prefix##_popback(name##_cont_t	*root)


/**
   Insert several items before the container first item. Items are
   pushed starting from the array first item. The function return
   pushed items count which can be lower than expected if the
   container is full.

   @param root container root
   @param item items array to insert
   @param size items count in array
   @return pushed item count
 */
#define __CONTAINER_PROTO_PUSH_ARRAY(name, prefix)		\
size_t								\
prefix##_push_array	(name##_cont_t *root,			\
			 name##_item_t *item,			\
			 size_t size)


/**
   Insert several items after the container last item. Items are
   pushed back starting from the array first item. The function return
   pushed items count which can be lower than expected if the
   container is full.

   @param root container root
   @param item items array to insert
   @param size items count in array
   @return pushed item count
 */
#define __CONTAINER_PROTO_PUSHBACK_ARRAY(name, prefix)		\
size_t								\
prefix##_pushback_array	(name##_cont_t *root,			\
			 name##_item_t *item,			\
			 size_t size)


/**
   Remove several items from the container head and copy them to an
   array. Items are poped to the array starting with first slot. The
   function return poped items count which can be lower than expected
   if the container become empty.

   @param root container root
   @param item items array to remove
   @param size slots count in array
   @return poped item count
 */
#define __CONTAINER_PROTO_POP_ARRAY(name, prefix)		\
size_t								\
prefix##_pop_array	(name##_cont_t	*root,			\
			 name##_item_t *item,			\
			 size_t size)


/**
   Remove several items from the container tail and copy them to an
   array. Items are poped to the array starting with first slot. The
   function return poped items count which can be lower than expected
   if the container become empty.

   @param root container root
   @param item items array to remove
   @param size slots count in array
   @return poped item count
 */
#define __CONTAINER_PROTO_POPBACK_ARRAY(name, prefix)		\
size_t								\
prefix##_popback_array	(name##_cont_t	*root,			\
			 name##_item_t *item,			\
			 size_t size)


/**
   Iterate over the whole container. Stop iteration if the iterator
   function return non zero.

   @param root container root
   @param fcn pointer to function called for each container item
   @param param context pointer passed to iterator function
   @return error code if any
 */
#define __CONTAINER_PROTO_FOREACH(name, prefix)			\
error_t								\
prefix##_foreach	(name##_cont_t *root,			\
			 error_t (*fcn) (name##_item_t i,	\
					 void *param),		\
			 void *param)


/**
   Init the container root

   @param root container root
   @return error code, zero on no error
 */
#define __CONTAINER_PROTO_INIT(name, prefix)			\
error_t								\
prefix##_init		(name##_cont_t *root)


/**
   Destroy the container root. Container must be empty, remaining
   items won't be destroyed here.

   @param root container root
 */
#define __CONTAINER_PROTO_DESTROY(name, prefix)			\
void								\
prefix##_destroy	(name##_cont_t *root)


/**
   Lock the container root for modification.

   @param root container root
 */
#define __CONTAINER_LOCKED_PROTO_WRLOCK(name, prefix)			\
void									\
prefix##_wrlock		(name##_cont_t *root)


/**
   Lock the container root for read.

   @param root container root
 */
#define __CONTAINER_LOCKED_PROTO_RDLOCK(name, prefix)			\
void									\
prefix##_rdlock		(name##_cont_t *root)


/**
   Unlock the container root.

   @param root container root
 */
#define __CONTAINER_LOCKED_PROTO_UNLOCK(name, prefix)			\
void									\
prefix##_unlock		(name##_cont_t *root)

/***********************************************************************
 *	User level container functions prototypes macro
 */

#define	CONTAINER_PROTOTYPE(attr, name, prefix)			\
								\
attr __CONTAINER_PROTO_ISNULL(name, prefix);			\
attr __CONTAINER_PROTO_GET(name, prefix);			\
attr __CONTAINER_PROTO_SET(name, prefix);			\
attr __CONTAINER_PROTO_NEXT(name, prefix);			\
attr __CONTAINER_PROTO_PREV(name, prefix);			\
attr __CONTAINER_PROTO_HEAD(name, prefix);			\
attr __CONTAINER_PROTO_TAIL(name, prefix);			\
attr __CONTAINER_PROTO_COUNT(name, prefix);			\
attr __CONTAINER_PROTO_MAXCOUNT(name, prefix);			\
attr __CONTAINER_PROTO_DELETE(name, prefix);			\
attr __CONTAINER_PROTO_PUSH(name, prefix);			\
attr __CONTAINER_PROTO_PUSHBACK(name, prefix);			\
attr __CONTAINER_PROTO_POP(name, prefix);			\
attr __CONTAINER_PROTO_POPBACK(name, prefix);			\
attr __CONTAINER_PROTO_PUSH_ARRAY(name, prefix);		\
attr __CONTAINER_PROTO_PUSHBACK_ARRAY(name, prefix);		\
attr __CONTAINER_PROTO_POP_ARRAY(name, prefix);			\
attr __CONTAINER_PROTO_POPBACK_ARRAY(name, prefix);		\
attr __CONTAINER_PROTO_FOREACH(name, prefix);			\
attr __CONTAINER_PROTO_INIT(name, prefix);			\
attr __CONTAINER_PROTO_DESTROY(name, prefix);

/***********************************************************************/

#define	__CONTAINER_LOCK_FUNC(attr, name, prefix, lockname)	\
								\
attr __CONTAINER_LOCKED_PROTO_WRLOCK(name, prefix)		\
{								\
  __cont_##lockname##_wrlock(&root->lock);			\
}								\
								\
attr __CONTAINER_LOCKED_PROTO_RDLOCK(name, prefix)		\
{								\
  __cont_##lockname##_rdlock(&root->lock);			\
}								\
								\
attr __CONTAINER_LOCKED_PROTO_UNLOCK(name, prefix)		\
{								\
  __cont_##lockname##_unlock(&root->lock);			\
}

/***********************************************************************
 *	User level container types and functions defintions macro
 */


/* empty lock macros */

typedef struct {} __cont_NOLOCK_type_t;
#define		__cont_NOLOCK_wrlock(lock)
#define		__cont_NOLOCK_rdlock(lock)
#define		__cont_NOLOCK_unlock(lock)
#define		__cont_NOLOCK_init(lock)		0
#define		__cont_NOLOCK_destroy(lock)


/**
   define type associated with a container.

   @param name is the new container name
   @param cont is the container algorithm used (CLIST, DLIST, VECTOR, ...)
   @param type is the user item C type handled by the container
   @param lockname is the lock policy used (NONE, SPIN, SPIN_IRQ, PTHREAD_MUTEX, ...)
 */

#define		CONTAINER_TYPE_DECL(name, cont, type, lockname, ...)	\
  __CONTAINER_##cont##_TYPE_DECL(name, type, lockname, __VA_ARGS__)

/**
   define all functions used to access the container

   @param attr can be used as function qualifer (static, static inline, ...)
   @param name is the container name
   @param cont is the container algorithm used (CLIST, DLIST, VECTOR, ...)
   @param prefix will be used a functions name prefix
   @param lockname is the lock policy used (NONE, SPIN, SPIN_IRQ, PTHREAD_MUTEX, ...)
 */

#define		CONTAINER_FUNC(attr, name, cont, prefix, lockname, ...)	\
  __CONTAINER_##cont##_FUNC(attr, name, prefix, lockname, __VA_ARGS__)		\
  __CONTAINER_LOCK_FUNC(attr, name, prefix, lockname)

#endif

