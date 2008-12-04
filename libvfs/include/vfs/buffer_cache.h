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

    UPMC / LIP6 / SOC (c) 2008
    Copyright Ghassan Almaless <ghassan.almaless@gmail.com>
*/

#ifndef __BUFFER_CACHE_
#define __BUFFER_CACHE_

#include <hexo/error.h>
#include <hexo/lock.h>
#include <mutek/scheduler.h>

#define BC_BUSY          1
#define BC_DELAYED_WRITE 2
#define BC_VALID         4

#define IS_BUFFER(state,flag)      (state) & (flag)
#define SET_BUFFER(state,flag)     (state) |= (flag)
#define CLEAR_BUFFER(state,flag)   (state) &= ~(flag)
#define INIT_BUFFER_STATE(state)   (state) = 0

typedef uint_fast32_t key_t;

struct bc_buffer_s
{
  key_t key1;
  key_t key2;
  uint_fast16_t state;
  uint_fast8_t index;
  sched_queue_root_t wait;
  struct bc_buffer_s *hash_next;
  struct bc_buffer_s *hash_pred;
  struct bc_buffer_s *freelist_next;
  struct bc_buffer_s *freelist_pred;
  void *content;
};

struct bc_request_s
{
  key_t key1;
  key_t key2;
  uint_fast8_t count;
  struct bc_buffer_s **buffers;
};

struct bc_entry_s
{
  struct bc_buffer_s *head;
};

struct buffer_cache_s;

typedef uint_fast16_t bc_hash_func_t(struct buffer_cache_s *bc, key_t key1,key_t key2);

struct buffer_cache_s
{
  uint_fast16_t entries_number;
  struct bc_entry_s *entries;
  bc_hash_func_t * bc_hash;
  size_t buffer_size;
};

struct bc_freelist_s
{
  sched_queue_root_t wait;
  struct bc_buffer_s *head;
};


extern struct buffer_cache_s bc;
extern struct bc_freelist_s freelist;



extern error_t bc_init(struct buffer_cache_s *bc,
		       struct bc_freelist_s *freelist,
		       uint_fast8_t entries_number,
		       uint_fast8_t buffers_per_entry,
		       size_t buffer_size,
		       bc_hash_func_t *hash_func);

/*
 * Caller have to check for BC_IO_ERROR, BC_DELAYED_WR and BC_DIRTY flags 
 * and to take the appropriate decision. whatever the decision is, caller 
 * have to clear flags in question.
 */
extern struct bc_request_s* bc_get_buffer(struct buffer_cache_s *bc, 
					  struct bc_freelist_s *freelist, 
					  struct bc_request_s *request);



extern struct bc_request_s* bc_release_buffer(struct buffer_cache_s *bc,
					      struct bc_freelist_s  *freelist,
					      struct bc_request_s *request,
					      bool_t hasError);

extern void bc_sync(struct buffer_cache_s *bc, 
		    struct bc_freelist_s *freelist);

extern uint_fast16_t bc_default_hash(struct buffer_cache_s *bc, 
				     key_t key1,
				     key_t key2);


#if BC_INSTRUMENT
extern uint_fast32_t dw_count;
extern uint_fast32_t hit_count;
extern uint_fast32_t miss_count;
extern uint_fast32_t sync_count;
extern uint_fast32_t wait_count;
#endif


#endif
