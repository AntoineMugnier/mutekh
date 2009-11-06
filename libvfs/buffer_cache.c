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

#include <mutek/mem_alloc.h>
#include <vfs/buffer_cache.h>
#include <device/block.h>

struct buffer_cache_s bc;
struct bc_freelist_s freelist;

#ifdef CONFIG_BC_INSTRUMENT
uint_fast32_t dw_count;
uint_fast32_t hit_count;
uint_fast32_t miss_count;
uint_fast32_t sync_count;
uint_fast32_t wait_count;
#endif

uint_fast16_t bc_default_hash(struct buffer_cache_s *bc, key_t key1, key_t key2)
{
  assert(bc != NULL);

  return (key2 % bc->entries_number);
}


error_t bc_init(struct buffer_cache_s *bc,
		struct bc_freelist_s *freelist,
		uint_fast8_t entries_number,
		uint_fast8_t buffers_per_entry,
		size_t buffer_size,
		bc_hash_func_t *hash_func)
{
  uint_fast16_t i,j;
  struct bc_entry_s *entries = NULL;
  struct bc_buffer_s *pred = NULL;
  struct bc_buffer_s *current = NULL;
  
  assert(bc != NULL);
  assert(freelist != NULL);
  assert(entries_number > 0);
  
  if((entries=mem_alloc(sizeof(*entries) * entries_number, mem_region_get_local(mem_scope_sys))) == NULL)
    return ENOMEM;
  
  sched_queue_init(&freelist->wait);

  for(i=0;i<entries_number;i++)
  {
    current = mem_alloc(sizeof(*current), mem_region_get_local(mem_scope_sys));
    if(current == NULL) return ENOMEM;
    
    current->key1 = -1;
    current->key2 = -1;
    INIT_BUFFER_STATE(current->state);
    current->index = i;
    sched_queue_init(&current->wait);
    current->content = mem_alloc(sizeof(uint8_t) * buffer_size, mem_region_get_local(mem_scope_sys));
   
    if(current->content == NULL)
      return ENOMEM;

    entries[i].head = current;

    if(pred != NULL)
    {
      pred->freelist_next = current;
      current->freelist_pred = pred;
    }
    pred = current;
    for(j=1;j<buffers_per_entry;j++)
    {
      current = mem_alloc(sizeof(struct bc_buffer_s), mem_region_get_local(mem_scope_sys));
      if(current == NULL) return ENOMEM;

      current->key1 = -1;
      current->key2 = -1;
      INIT_BUFFER_STATE(current->state);
      current->index = i;
      sched_queue_init(&current->wait);
      current->content = mem_alloc(sizeof(uint8_t) * buffer_size, mem_region_get_local(mem_scope_sys));
      
      if(current->content == NULL)
          return ENOMEM;
      
      pred->hash_next = current;
      current->hash_pred = pred;
      
      pred->freelist_next = current;
      current->freelist_pred = pred;
      pred=current;
    }
    entries[i].head->hash_pred = current;
    current->hash_next = entries[i].head;
  }

  freelist->head = entries[0].head;
  freelist->head->freelist_pred = current;
  current->freelist_next = freelist->head;
  
  bc->entries_number = entries_number;
  bc->entries = entries;
  bc->bc_hash = hash_func;  
  bc->buffer_size = buffer_size;
  return 0;
}

static struct bc_buffer_s* bc_find(struct buffer_cache_s *bc, 
				   uint_fast16_t index, 
				   key_t key1, 
				   key_t key2)
{
  assert(bc != NULL);
  
  struct bc_buffer_s *current=bc->entries[index].head;

  if((current == NULL) || ((current->key1 == key1) && (current->key2 == key2))){
#ifdef CONFIG_BC_DEBUG
    if(current!=NULL)
      printk("bc_find: found buffer key1 %d, key2 %d\n",key1,key2);
#endif
    return current;
  }

#ifdef CONFIG_BC_DEBUG
  printk("bc_find: first buffer key1 %d, key2 %d\n",current->key1,current->key2);
#endif

  for(current = current->hash_next; current != bc->entries[index].head; current = current->hash_next)
    if((current->key1 == key1) && (current->key2 == key2))
      return current;
  
  return NULL;
}

static struct bc_buffer_s* bc_hash_link(struct buffer_cache_s *bc, 
					struct bc_buffer_s *buffer)
{
  assert(bc != NULL);
  assert(buffer != NULL);
  
  uint_fast16_t index = buffer->index;
  struct bc_buffer_s *head = bc->entries[index].head;
  
  buffer->hash_next = (head == NULL) ? buffer : head;
  buffer->hash_pred = (head == NULL) ? buffer : head->hash_pred;
  
  buffer->hash_pred->hash_next = buffer;
  buffer->hash_next->hash_pred = buffer;

  bc->entries[index].head = buffer;

  return buffer;
}


static struct bc_buffer_s* bc_hash_unlink(struct buffer_cache_s *bc, 
					  struct bc_buffer_s *buffer)
{
  assert(bc != NULL);
  assert(buffer != NULL);

  uint_fast16_t index = buffer->index;
  assert(bc->entries[index].head != NULL);
  
  struct bc_buffer_s *head = bc->entries[index].head;
  
  if((buffer == head)&&(head->hash_next == head))
  {
#ifdef CONFIG_BC_DEBUG
    printk("bc_hash_unlink: buffer %d is last element in the queue %d\n",
	   buffer->key2,index);
#endif

    bc->entries[index].head = NULL;
    goto BC_HASH_UNLINK_END;
  }

  if(head == buffer)
    bc->entries[index].head = buffer->hash_next;

  buffer->hash_next->hash_pred = buffer->hash_pred;
  buffer->hash_pred->hash_next = buffer->hash_next;

 BC_HASH_UNLINK_END:
  buffer->hash_next = NULL;
  buffer->hash_pred = NULL;

  return buffer;
}


static struct bc_buffer_s* bc_freelist_link(struct bc_freelist_s *freelist, 
					    struct bc_buffer_s *buffer, 
					    bool_t isFirst)
{
  assert(freelist != NULL);
  assert(buffer != NULL);
  
  struct bc_buffer_s *head = freelist->head;
  
  if(head == NULL)
  {
    
#ifdef CONFIG_BC_DEBUG
    printk("bc_freelist_link: buffer %d is put in head of list\n",
	   buffer->key2);
#endif

    freelist->head = buffer;
    buffer->freelist_next = buffer;
    buffer->freelist_pred = buffer;
    return buffer;
  }
  
  buffer->freelist_next = head;
  buffer->freelist_pred = head->freelist_pred;
  
  head->freelist_pred->freelist_next = buffer;
  head->freelist_pred = buffer;
  
  if(isFirst) freelist->head = buffer;

  return buffer;
}



static struct bc_buffer_s* bc_freelist_unlink(struct bc_freelist_s *freelist, 
					      struct bc_buffer_s *buffer)
{
  assert(freelist != NULL);
  assert(buffer != NULL);
  assert(freelist->head != NULL);

  struct bc_buffer_s *head = freelist->head;
  
  if((head == buffer)&&(head->freelist_next == head))
  {
    freelist->head = NULL;
    goto BC_FREELIST_UNLINK_END;
  }

  if(head == buffer)
    freelist->head = buffer->freelist_next;
 
  buffer->freelist_pred->freelist_next = buffer->freelist_next;
  buffer->freelist_next->freelist_pred = buffer->freelist_pred;

 BC_FREELIST_UNLINK_END:
  buffer->freelist_next = NULL;
  buffer->freelist_pred = NULL;

  return buffer;
}


static struct bc_buffer_s* bc_freelist_get(struct bc_freelist_s *freelist)
{
  assert(freelist != NULL);
  
  struct bc_buffer_s *head = freelist->head;
  struct bc_buffer_s *buffer = NULL;
  
  if(head == NULL)
    return NULL;

  buffer = head;

  if((head->freelist_next == head))
  {
    freelist->head = NULL;
    goto BC_FREELIST_GET_END;
  }
  
  head->freelist_pred->freelist_next = head->freelist_next;
  head->freelist_next->freelist_pred = head->freelist_pred;

  freelist->head = head->freelist_next;

 BC_FREELIST_GET_END:
  buffer->freelist_next = NULL;
  buffer->freelist_pred = NULL;
  
  return buffer;
}


struct bc_request_s* bc_get_buffer(struct buffer_cache_s *bc, 
				  struct bc_freelist_s *freelist, 
				  struct bc_request_s *request)
{
  assert(bc != NULL);
  assert(freelist != NULL);
  assert(request != NULL);

  struct bc_buffer_s *buffer;
  uint8_t *data[1];
  uint_fast16_t index; 
  reg_t interrupt_state;
  uint_fast8_t count;
  int_fast16_t i;
  error_t err;
  key_t key1;
  key_t key2;
  
  buffer = NULL;
  count = request->count;
  key1 = request->key1;
  key2 = request->key2;
  err = 0;
  i = 0;

  cpu_interrupt_savestate_disable(&interrupt_state);
  sched_queue_wrlock(&freelist->wait);         /* <-- */

  while( i < count)
  {
    index = bc->bc_hash(bc,key1,key2); 
#ifdef CONFIG_BC_DEBUG
    printk("bc_get_buffer: i %d, count %d, index %d, key1 %d, key2 %d\n",
	   i,count,index, key1,key2);
#endif
    if((buffer=bc_find(bc,index,key1,key2)) == NULL)
    { 
#ifdef CONFIG_BC_DEBUG
      printk("bc_get_buffer: buffer not found in the cache\n");
#endif
      if((buffer=bc_freelist_get(freelist)) == NULL)
      {
#ifdef CONFIG_BC_DEBUG
	printk("going to sleep, no more free buffers\n");
#endif
	sched_wait_unlock(&freelist->wait);       /* --> */
	sched_queue_wrlock(&freelist->wait);      /* <-- */
	continue;
      }

    
      SET_BUFFER(buffer->state,BC_BUSY);
      
      if(IS_BUFFER(buffer->state, BC_DELAYED_WRITE))
      {
	sched_queue_unlock(&freelist->wait);     /* --> */
	cpu_interrupt_restorestate(&interrupt_state);

	request->buffers[i] = buffer;
	CLEAR_BUFFER(buffer->state,BC_DELAYED_WRITE);

	data[0] = buffer->content;

	if(dev_block_wait_write((struct device_s *)buffer->key1, data, buffer->key2, 1))
	{
	  request->count = i + 1;
	  return NULL;
	}
#ifdef CONFIG_BC_INSTRUMENT
	dw_count ++;
#endif
#ifdef CONFIG_BC_DEBUG
	printk(">>> Delayed write %d\n",buffer->key2);
#endif
	
	sched_queue_wrlock(&freelist->wait);     /* <-- */
	cpu_interrupt_savestate_disable(&interrupt_state);
	bc_freelist_link(freelist,buffer,1);
	while(sched_wake(&freelist->wait));
	while(sched_wake(&buffer->wait));
	CLEAR_BUFFER(buffer->state,BC_BUSY);
	continue;
      }
      
      bc_hash_unlink(bc,buffer);
      buffer->key1 = key1;
      buffer->key2 = key2;
      buffer->index = index;
      bc_hash_link(bc,buffer);
      
      CLEAR_BUFFER(buffer->state,BC_VALID);
      request->buffers[i] = buffer;
#ifdef CONFIG_BC_INSTRUMENT
      miss_count ++;
#endif

#ifdef CONFIG_BC_DEBUG
      printk(">> MISS %d\n",key2);
#endif
      i++; key2++;
      continue;
    }
   
    if(IS_BUFFER(buffer->state,BC_BUSY))
    {
#ifdef CONFIG_BC_INSTRUMENT
      wait_count ++;
#endif
      sched_wait_callback(&buffer->wait, (sched_wait_cb_t*)sched_queue_unlock, &freelist->wait); /* --> */
      sched_queue_wrlock(&freelist->wait);     /* <-- */
      continue;
    }
    
    SET_BUFFER(buffer->state,BC_BUSY);
#ifdef CONFIG_BC_INSTRUMENT
    hit_count ++;
#endif

#ifdef CONFIG_BC_DEBUG
    printk(">> Hit %d, D_W %s\n",key2,(IS_BUFFER(buffer->state,BC_DELAYED_WRITE)) ? "Yes" : "No");
#endif
    bc_freelist_unlink(freelist,buffer);
  
    request->buffers[i] = buffer;
    i++; key2++;
  }

  cpu_interrupt_restorestate(&interrupt_state);
  sched_queue_unlock(&freelist->wait);

  return request;
}


struct bc_request_s* bc_release_buffer(struct buffer_cache_s *bc,
				       struct bc_freelist_s  *freelist,
				       struct bc_request_s *request,
				       bool_t hasError)
{ 
  uint_fast8_t i;
  struct bc_buffer_s *buffer;
  
  assert(bc != NULL);
  assert(freelist != NULL);
  assert(request != NULL);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  sched_queue_wrlock(&freelist->wait);

  for(i=0; i < request->count; i++)
  {
    buffer= request->buffers[i];
#ifdef CONFIG_BC_DEBUG
    printk("bc_release_buffer: i %d, count %d, key1 %d, key2 %d\n",
	   i,request->count,buffer->key1,buffer->key2);
#endif
    assert(IS_BUFFER(buffer->state, BC_BUSY));
    CLEAR_BUFFER(buffer->state, BC_BUSY);

    if(hasError)
      CLEAR_BUFFER(buffer->state,BC_VALID | BC_DELAYED_WRITE);
    else
      SET_BUFFER(buffer->state,BC_VALID);
   
    bc_freelist_link(freelist,buffer,hasError);
  }
  
  while(sched_wake(&freelist->wait));
  while(sched_wake(&buffer->wait));
  sched_queue_unlock(&freelist->wait);
  
  CPU_INTERRUPT_RESTORESTATE;
  return request;
}



void bc_sync(struct buffer_cache_s *bc, struct bc_freelist_s *freelist)
{
  struct bc_buffer_s *current=NULL;
  uint_fast16_t i;
#ifdef CONFIG_BC_INSTRUMENT
  uint_fast32_t tm_tmp;
  uint_fast32_t tm_now;
  uint_fast32_t count;
  uint_fast32_t tm_total;
#endif
  uint8_t *data[1];
 
#ifdef CONFIG_BC_INSTRUMENT
  tm_total = 0;
  count = 0;
  tm_tmp = tm_now = 0;
#endif

  sched_queue_wrlock(&freelist->wait);      /* <-- */
  for(i=0; i< bc->entries_number; i++)
  {
    if((current = bc->entries[i].head) != NULL)
    {
      do
      {
	if(IS_BUFFER(current->state, BC_DELAYED_WRITE))
	{
#ifdef CONFIG_BC_INSTRUMENT
	  printk(">>>>> Sync: %d",current->key2);
#else
# ifdef CONFIG_BC_DEBUG  
	  printk(">>>>> Sync: %d\n",current->key2);
# endif
#endif
	  data[0] = current->content;
#ifdef CONFIG_BC_INSTRUMENT
	  sync_count ++;
	  tm_tmp = cpu_cycle_count();
#endif
	  if(dev_block_wait_write((struct device_s *)current->key1, data, current->key2, 1))
	    printk("bc_sync: I/O error while writing bloc %d\n",current->key2);
#ifdef CONFIG_BC_INSTRUMENT
	  tm_now = cpu_cycle_count() - tm_tmp;
	  tm_tmp = tm_now;
	  tm_total += tm_now;
	  count ++;
#endif
#ifdef CONFIG_BC_INSTRUMENT
	  printk(", tm_stamp %d\n",tm_now);
#endif
	}
	current = current->hash_next;
      }while(current != bc->entries[i].head);
    }
  }
  sched_queue_unlock(&freelist->wait);     /* --> */
    
#ifdef CONFIG_BC_INSTRUMENT
  if(count)
    printk(">> Sync: %d blk, average latency %d cycles\n",count, tm_total/count);
#endif 
}


/* =================================================================== */

void bc_dump(struct buffer_cache_s *bc)
{
  uint_fast16_t i;
  struct bc_buffer_s *current=NULL;
  for(i=0; i< bc->entries_number; i++)
  {
    if((current = bc->entries[i].head) != NULL){
      printk("keys in entry #%d are [%d,",i,current->key2);
      for(current = current->hash_next; current != bc->entries[i].head; current = current->hash_next)
	printk("%d,",current->key2);
      printk("\b]\n");
    }
    else
      printk("keys in entry %d are empty\n",i);
  }
  printk("\n");
}


void bc_dump2(struct bc_freelist_s *freelist)
{
  struct bc_buffer_s *current=NULL;
  
  int8_t i=0;
  current = freelist->head;
  if(current == NULL) {printk("[empty]\n"); return;}

  printk("[(%d,%d),",current->index,current->key2);
  for(current = current->freelist_next; current != freelist->head; current = current->freelist_next)
    {
      printk("(%d,%d),",current->index,current->key2);
              if(i== 60)
	break;
      i++;
      
    }
  printk("\b]\n");
}


void bc_dump3(struct bc_freelist_s *freelist)
{
  struct bc_buffer_s *current=NULL;

  if(freelist->head == NULL) {printk("[empty]\n"); return;}

  printk("[");
  
  for(current = freelist->head->freelist_pred; current != freelist->head; current = current->freelist_pred)
    printk("%d,",current->key2);

  printk("%d]\n",freelist->head->key2);
}
