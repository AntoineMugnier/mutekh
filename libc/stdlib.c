
#include <stdlib.h>
#include <stdio.h>

void exit(uint_fast8_t status)
{
  printk("called exit() with %i. aborting ...", status);
  abort();
}

error_t atexit(void (*function)(void))
{
  /* FIXME */
  return -1;
}

