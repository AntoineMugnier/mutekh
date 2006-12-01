
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

CONTAINER_FUNC(static inline, stream_fifo, RING, stream_fifo, NOLOCK);

error_t	fclose(FILE *stream)
{
  error_t	err;

  err = fflush(stream);
  err |= stream->ops->close(stream->fd);

  free(stream);

  return (err);
}

static error_t	write_flush(FILE *stream)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  size_t	size = stream_fifo_pop_array(&stream->fifo_write, local, CONFIG_LIBC_STREAM_BUFFER_SIZE);
  ssize_t	res;

  /* write data from buffer */
  do
    {
      res = stream->ops->write(stream->fd, local, size);

      if (res < 0)
	return res;

      size -= res;
    }
  while (size);

  return 0;
}

static error_t	read_flush(FILE *stream)
{
  stream->ops->lseek(stream->fd, stream->pos, SEEK_CUR);

  stream_fifo_clear(&stream->fifo_read);

  return 0;
}

static error_t	no_flush(FILE *stream)
{
  return 0;
}

static ssize_t	buffered_read(size_t size_, FILE *stream, uint8_t *ptr)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  ssize_t	res, local_size;
  size_t	size = size_;

  if (!stream->ops->readable(stream->fd))
    return -1;

  stream->rwflush = &read_flush;

  /* get data from buffer */
  res = stream_fifo_pop_array(&stream->fifo_read, ptr, size);
  size -= res;
  ptr += res;

  /* read more data directly from fd */
  while (size > CONFIG_LIBC_STREAM_BUFFER_SIZE)
    {
      res = stream->ops->read(stream->fd, ptr, CONFIG_LIBC_STREAM_BUFFER_SIZE);

      if (res < 0)
	return res;

      if (res == 0)
	break;

      size -= res;
      ptr += res;
    }

  /* read remaining data in local buffer and dispatch */
  for (local_size = 0; size > 0; )
    {
      res = stream->ops->read(stream->fd, local + local_size,
			      CONFIG_LIBC_STREAM_BUFFER_SIZE - local_size);

      if (res < 0)
	return res;

      if (res == 0)
	break;

      local_size += res;

      /* if enough data, dispatch */
      if (local_size >= size)
	{
	  memcpy(ptr, local, size);
	  stream_fifo_pushback_array(&stream->fifo_read, local + size, local_size - size);
	  break;
	}
    }

  stream->pos += size_;

  return size_ - size;
}

static error_t	buffered_write(size_t size_, FILE *stream, const uint8_t *ptr)
{
  size_t	size = size_;

  if (!stream->ops->writable(stream->fd))
    return -1;

  stream->rwflush = &write_flush;

  /* check to see if all data can be put in buffer */
  if (stream_fifo_count(&stream->fifo_write) + size <= CONFIG_LIBC_STREAM_BUFFER_SIZE)
    {
      stream_fifo_pushback_array(&stream->fifo_write, ptr, size);
    }
  else
    {
      ssize_t	res;

      /* write all data present in buffer */
      write_flush(stream);

      /* write data directly to device if greater than buffer */
      while (size > CONFIG_LIBC_STREAM_BUFFER_SIZE)
	{
	  res = stream->ops->write(stream->fd, ptr, size);

	  if (res < 0)
	    return res;

	  size -= res;
	  ptr += res;
	}

      /* fill buffer with remaining data */
      stream_fifo_pushback_array(&stream->fifo_write, ptr, size);
    }

  stream->pos += size_;

  return 0;
}

error_t	fflush(FILE *stream)
{
  assert(stream != NULL);

  return write_flush(stream);
}

error_t	fpurge(FILE *stream)
{
  return read_flush(stream);
}

size_t	fread(void *ptr_, size_t size,
	      size_t nmemb, FILE *stream)
{
  uint8_t	*ptr = ptr_;
  size_t	i;

  for (i = 0; i < nmemb; i++)
    {
      if (buffered_read(size, stream, ptr) <= 0)
	break;

      ptr += size;
    }

  return i;
}

size_t	fwrite(const void *ptr_, size_t size, size_t nmemb, FILE *stream)
{
  uint8_t	*ptr = ptr_;
  size_t	i;

  for (i = 0; i < nmemb; i++)
    {
      if (buffered_write(size, stream, ptr))
	break;

      ptr += size;
    }

  return i;
}

inline error_t fseek(FILE *stream, fpos_t offset, enum stream_whence_e whence)
{
  stream->rwflush(stream);

  if (stream->ops->lseek(stream->fd, offset, whence) >= 0)
    return 0;
  else
    return (EOF);
}

fpos_t ftell(FILE *stream)
{
  return stream->pos;
}

void rewind(FILE *stream)
{
  fseek(stream, 0, SEEK_SET);
}

error_t fgetpos(FILE *stream, fpos_t *pos)
{
  *pos = stream->pos;
  return 0;
}

error_t	fsetpos(FILE *stream, const fpos_t *pos)
{
  return fseek(stream, *pos, SEEK_SET);
}

uint_fast16_t fgetc(FILE *stream)
{
  unsigned char	res;

  if (buffered_read(1, stream, &res) <= 0)
    return EOF;
  else
    return res;
}

char	*fgets(char *str_, size_t size, FILE *stream)
{
  uint8_t	res;
  char	*str = str_;
  char	*ret = NULL;

  while (size-- > 1)
    {
      if (buffered_read(1, stream, &res))
	break;

      *str++ = res;
      ret = str_;

      if (res == '\n')
	break;
    }

  *str = 0;

  return ret;
}

uint_fast16_t fputc(unsigned char c, FILE *stream)
{
  if (buffered_write(1, stream, &c))
    return EOF;

  return (c);
}

error_t	fputs(const char *str, FILE *stream)
{
  return (buffered_write(strlen(str), stream, str));
}

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

FILE	*fopen(const char *path, const char *mode)
{
  FILE		*stream;
  mode_t	flags;

  if ((flags = open_mode(mode)) < 0)
    goto err;

  if (!(stream = malloc(sizeof (FILE))))
    goto err;

  stream->ops = stream_fops;

  if (!(stream->fd = stream->ops->open(path, flags)))
    goto err_1;

  stream->rwflush = &no_flush;
  stream->pos = stream->ops->lseek(stream->fd, 0, SEEK_CUR);
  stream_fifo_init(&stream->fifo_read);
  stream_fifo_init(&stream->fifo_write);

  return (stream);

 err_1:
  free(stream);
 err:
  return NULL;
}

