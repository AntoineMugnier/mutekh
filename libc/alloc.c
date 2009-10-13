
#include <hexo/alloc.h>

#include <string.h>
#include <stdlib.h>

inline void *
malloc(size_t size)
{
  return mem_alloc(size, MEM_SCOPE_DEFAULT);
}

void *
calloc(size_t nmemb, size_t size)
{
  void	*ptr;

  if ((ptr = malloc(nmemb * size)))
    memset(ptr, 0, nmemb * size);

  return ptr;
}

void
free(void *ptr)
{
  if (ptr != NULL)
    mem_free(ptr);
}

void *
realloc(void *ptr, size_t size)
{
  size_t	oldsize;
  void		*p;

  if (ptr == NULL)
    return malloc(size);

  oldsize = mem_alloc_getsize(ptr);

  if (oldsize >= size)
    return ptr;

  
  if( ! (p = malloc(size)))
    return NULL;

  memcpy(p, ptr, oldsize);

  free(ptr);
  return p;
}

