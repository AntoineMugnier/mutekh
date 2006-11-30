
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

/*

%config CONFIG_LIBC_STREAM
desc enable C library file streaming functions (fopen, fclose, ...)
require CONFIG_LIBC_STREAM_BUFFER_SIZE
%config end

%config CONFIG_LIBC_STREAM_BUFFER_SIZE
desc C library default buffer size for streams
parent CONFIG_LIBC_STREAM
default 8192
%config end

*/

const struct stream_ops_s	*stream_fops;

/************************************************************************/

error_t	fclose(FILE *stream)
{
  error_t	err;

  err = fflush(stream);
  err |= stream->ops->close(stream->fd);

  free(stream->buffer);
  free(stream);

  return (err);
}

static error_t	write_flush(FILE *stream)
{
  ssize_t	err;

  if (stream->length)
    {
      err = stream->ops->write(stream->fd, stream->buffer, stream->length);

      if (err < stream->length)
	return -1;

      stream->length = 0;
    }

  return 0;
}

static size_t	buffered_read(size_t size, FILE *stream, char **ptr)
{
  ssize_t	tmp;
  size_t	pos;
  size_t	init_size;

  init_size = size;

  if (!stream->length)
    {
      if ((tmp = stream->ops->read(stream->fd, stream->buffer, stream->buf_size) < 0))
	return -1;

      stream->length = tmp;
      pos = 0;
    }
  else
    pos = stream->buf_size - stream->length;

  while (stream->length && size)
    {
      *(*ptr)++ = stream->buffer[pos++];
      stream->length--;
      size--;
    }

  return (init_size - size);
}

static error_t	buffered_write(size_t size, FILE *stream, const char *ptr)
{
  intptr_t i = 0;

  if (size >= stream->buf_size)
    {
      if (write_flush(stream))
	return -1;

      while (size >= stream->buf_size)
	{
	  if (stream->ops->write(stream->fd, ptr, stream->buf_size) < stream->buf_size)
	    return -1;

	  ptr += stream->buf_size;
	  size -= stream->buf_size;
	}
    }

  while (size)
    {
      if ((stream->length >= stream->buf_size) && write_flush(stream))
	return -1;

      stream->buffer[stream->length++] = ptr[i++];
      size--;
    }

  return 0;
}

error_t	fflush(FILE *stream)
{
  error_t		err;

  assert(stream != NULL);

  switch (stream->last)
    {
    case (STREAM_LAST_READ):
      err = stream->ops->lseek(stream->fd, -stream->length, SEEK_CUR);
      stream->length = 0;

      if (err < 0)
	return (EOF);
      break;

    case (STREAM_LAST_WRITE):
      return (write_flush(stream));

    default:
      break;
    }

  return 0;
}

error_t	fpurge(FILE *stream)
{
  stream->length = 0;

  return 0;
}

size_t		fread(void *ptr, size_t size,
		      size_t nmemb, FILE *stream)
{
  size_t	rsize;
  size_t	tmp;
  size_t	red;
  char		*ptr_;

  ptr_ = ptr;
  red = 0;

  if (stream->last != STREAM_LAST_READ)
    {
      fflush(stream);
      stream->last = STREAM_LAST_READ;
    }

  rsize = size * nmemb;

  while (1)
    {
      red += (tmp = buffered_read(rsize, stream, &ptr_));

      if (tmp < 0)
	return -1;

      if (!(rsize -= tmp) || (!tmp))
	break;
    }

  return (red / size);
}

size_t	fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream)
{
  if (stream->last != STREAM_LAST_WRITE)
    {
      fflush(stream);
      stream->last = STREAM_LAST_WRITE;
    }

  if (buffered_write(size * nmemb, stream, ptr))
    return -1;

  return (nmemb);
}

error_t fseek(FILE *stream, fpos_t offset, enum stream_whence_e whence)
{
  if (stream->ops->lseek(stream->fd, offset, whence) >= 0)
    return 0;
  else
    return (EOF);
}

fpos_t ftell(FILE *stream)
{
  return (stream->ops->lseek(stream->fd, 0, SEEK_CUR));
}

void rewind(FILE *stream)
{
  stream->ops->lseek(stream->fd, 0, SEEK_SET);
}

error_t fgetpos(FILE *stream, fpos_t *pos)
{
  if ((*pos = stream->ops->lseek(stream->fd, 0, SEEK_CUR)) >= 0)
    return 0;
  else
    return (EOF);
}

error_t	fsetpos(FILE *stream, const fpos_t *pos)
{
  if (stream->ops->lseek(stream->fd, *pos, SEEK_SET) >= 0)
    return 0;
  else
    return (EOF);
}

uint_fast16_t fgetc(FILE *stream)
{
  unsigned char	res;
  size_t res2;

  res2 = fread(&res, 1, 1, stream);

  if (res2 > 0)
    return res;
  else
    return EOF;
}

char	*fgets(char *str, size_t size, FILE *stream)
{
  char c;
  bool_t done = 0;
  size_t len = 0;

  while (--size && (c != '\n'))
    {
      len = fread(&c, 1, 1, stream);

      if (len <= 0)
	{
	  if (!done || (len < 0))
	    return NULL;
	  else
	    break;
	}

      *str++ = c;
      done = 1;
    }

  *str = 0;

  return (str);
}

uint_fast16_t fputc(unsigned char c, FILE *stream)
{
  if (fwrite(&c, 1, 1, stream) < 1)
    return EOF;

  return (c);
}

error_t	fputs(const char *str, FILE *stream)
{
  return (fwrite(str, strlen(str), 1, stream) > 0 ? 1 : EOF);
}

#if 0

static mode_t	open_mode(const char *str)
{
  mode_t	mode = 0666;

  while (*str)
    {
      switch (*str)
	{
	case ('r'):
	  mode |= O_RDONLY;
	  break;

	case ('w'):
	  mode |= O_WRONLY | O_CREAT | O_TRUNC;
	  break;

	case ('+'):
	  mode |= O_RDONLY | O_WRONLY;
	  break;

	case ('a'):
	  mode |= O_WRONLY | O_APPEND | O_CREAT;
	  break;

	case ('b'):
	  break;

	default:
	  return -1;
	}
      str++;
    }

  return (mode);
}

#endif

FILE	*fopen(const char *path, const char *mode)
{
  FILE	*stream;

  if (!(stream = malloc(sizeof (FILE))))
    goto err;

  stream->ops = stream_fops;
  stream->buf_size = CONFIG_LIBC_STREAM_BUFFER_SIZE;

  stream->buffer = malloc(stream->buf_size);
  if (!(stream->buffer))
    goto err_1;

  if (!(stream->fd = stream->ops->open(path, mode)))
    goto err_2;

  stream->length = 0;
  stream->last = STREAM_LAST_NONE;

  return (stream);

 err_2:
  free(stream->buffer);
 err_1:
  free(stream);
 err:
  return NULL;
}

