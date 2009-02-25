
#include <string.h>
#include <errno.h>

error_t errno = 0;

char *strerror(error_t errnum)
{
  return errnum ? "Error" : "Success"; /* FIXME :) */
}

