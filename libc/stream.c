
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

CONTAINER_FUNC(stream_fifo, RING, static inline, stream_fifo);

error_t	fclose(FILE *stream)
{
  error_t	err;

  err =  stream->rwflush(stream);
  err |= stream->ops->close(stream->fd);

  free(stream);

  return (err);
}

static error_t	no_flush(FILE *stream)
{
  return 0;
}

static error_t	write_flush(FILE *stream)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  uint8_t	*ptr = local;
  size_t	size = stream_fifo_pop_array(&stream->fifo_write, ptr, CONFIG_LIBC_STREAM_BUFFER_SIZE);
  ssize_t	res;

  /* write remaining data present in buffer */
  while (size)
    {
      res = stream->ops->write(stream->fd, ptr, size);

      if (res < 0)
	{
	  stream_fifo_pushback_array(&stream->fifo_write, ptr, size);
	  return res;
	}

      size -= res;
      ptr += res;
    };

  /* write buffer is empty here */
  stream->rwflush = &no_flush;
  return 0;
}

static error_t	read_flush(FILE *stream)
{
  /* seek fd to real pos and flush prefetched data in read buffer */
  stream->ops->lseek(stream->fd, stream->pos, SEEK_SET);
  stream_fifo_clear(&stream->fifo_read);

  /* read buffer is empty here */
  stream->rwflush = &no_flush;
  return 0;
}

error_t	__stdio_read(size_t size, FILE *stream, uint8_t *ptr)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  ssize_t	res, local_size;

  if (!stream->ops->readable(stream->fd))
    return -EINVAL;

  /* get data from buffer */
  res = stream_fifo_pop_array(&stream->fifo_read, ptr, size);
  size -= res;
  ptr += res;
  stream->pos += res;

  if (size)
    {
      /* read buffer is empty here */
      stream->rwflush = &no_flush;

      /* read more data directly from fd */
      while (size > CONFIG_LIBC_STREAM_BUFFER_SIZE)
	{
	  res = stream->ops->read(stream->fd, ptr, CONFIG_LIBC_STREAM_BUFFER_SIZE);

	  if (res <= 0)
	    return res;

	  size -= res;
	  ptr += res;
	  stream->pos += res;
	}
    }

  /* read remaining data in local buffer */
  for (local_size = 0; local_size < size; local_size += res)
    {
      res = stream->ops->read(stream->fd, local + local_size,
			      CONFIG_LIBC_STREAM_BUFFER_SIZE - local_size);

      if (res < 0)
	return res;
      if (res == 0)
	break;
    }

  memcpy(ptr, local, size);
  stream->pos += size;

  if (local_size >= size)
    {
      if (local_size > size)
	{
	  /* if more data than needed, put in read buffer */
	  stream_fifo_pushback_array(&stream->fifo_read, local + size, local_size - size);
	  stream->rwflush = &read_flush;
	}
      return 1;
    }
  else
    {
      /* not enough data have been read */
      return 0;
    }
}

static error_t unbuffered_write(size_t size, FILE *stream, const uint8_t *ptr)
{
  ssize_t res;

  while (size)
    {
      res = stream->ops->write(stream->fd, ptr, size);

      if (res < 0)
	return res;

      stream->pos += res;
      size -= res;
      ptr += res;
    }

  return 0;
}

error_t __stdio_write(size_t size, FILE *stream, const uint8_t *ptr)
{
  if (!stream->ops->writable(stream->fd))
    return -EINVAL;

  switch (stream->buf_mode)
    {
    case _IONBF:
      return unbuffered_write(size, stream, ptr);

    case _IOLBF: {
      ssize_t i;

      /* write all ended lines if any */
      for (i = size; i > 0; i--)
	if (ptr[i-1] == '\n')
	  {
	    ssize_t res;

	    if ((res = write_flush(stream)))
	      return res;
	    if ((res = unbuffered_write(i, stream, ptr)))
	      return res;

	    ptr += i;
	    size -= i;
	    break;
	  }
    }

    /* remaining data without end of line will be treated as block */

    case _IOFBF:

      /* check if all data can be put in buffer */
      if (stream_fifo_count(&stream->fifo_write) + size > CONFIG_LIBC_STREAM_BUFFER_SIZE)	
	{
	  ssize_t	res;

	  /* write all data present in buffer */
	  if ((res = write_flush(stream)))
	    return res;

	  /* write data directly to device if greater than buffer */
	  while (size > CONFIG_LIBC_STREAM_BUFFER_SIZE)
	    {
	      res = stream->ops->write(stream->fd, ptr, size);

	      if (res < 0)
		return res;

	      size -= res;
	      ptr += res;
	      stream->pos += res;
	    }
	}

      /* fill buffer with remaining data */
      stream_fifo_pushback_array(&stream->fifo_write, (uint8_t*)ptr, size);
      stream->pos += size;
      stream->rwflush = &write_flush;
    }

  return 0;
}

error_t	fflush(FILE *stream)
{
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
      if (__stdio_read(size, stream, ptr) <= 0)
	break;

      ptr += size;
    }

  return i;
}

size_t	fwrite(const void *ptr_, size_t size, size_t nmemb, FILE *stream)
{
  const uint8_t	*ptr = ptr_;
  size_t	i;

  for (i = 0; i < nmemb; i++)
    {
      if (__stdio_write(size, stream, ptr))
	break;

      ptr += size;
    }

  return i;
}

error_t fseek(FILE *stream, fpos_t offset, int_fast8_t whence)
{
  stream->rwflush(stream);

  if ((stream->pos = stream->ops->lseek(stream->fd, offset, whence)) >= 0)
    return 0;
  else
    return (EOF);
}

int_fast16_t fgetc(FILE *stream)
{
  unsigned char	res;
  return __stdio_read(1, stream, &res) <= 0 ? EOF : res;
}

int_fast16_t ungetc(int_fast16_t c, FILE *stream)
{
  return stream_fifo_pushback(&stream->fifo_read, c) ? c : EOF;
}

char *fgets(char *str_, size_t size, FILE *stream)
{
  char	*str = str_;
  char	*ret = NULL;

  while (size-- > 1)
    {
      error_t res = __stdio_read(1, stream, str);

      if (res == 0)
	break;
      else if (res < 0)
	return NULL;

      ret = str_;

      if (*str++ == '\n')
	break;
    }

  *str = 0;

  return ret;
}

int_fast16_t fputc(unsigned char c, FILE *stream)
{
  if (__stdio_write(1, stream, &c))
    return EOF;

  return (c);
}

error_t	fputs(const char *str, FILE *stream)
{
  return __stdio_write(strlen(str), stream, (uint8_t*)str) ? EOF : 0;
}

error_t puts(const char *str)
{
  if (fputs(str, stdout))
    return EOF;
  return fputc('\n', stdout) < 0 ? EOF : 0;
}

static uint_fast8_t	open_mode(const char *str)
{
  uint_fast8_t	mode = 0;

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

static const struct stream_ops_s *stream_fops;

void fopen_setops(const struct stream_ops_s *ops)
{
  stream_fops = ops;
}

FILE *fopen(const char *path, const char *mode)
{
  FILE		*stream;
  uint_fast8_t	flags;

  if ((flags = open_mode(mode)) < 0)
    goto err;

  if (!(stream = malloc(sizeof (FILE))))
    goto err;

  stream->ops = stream_fops;

  if (!(stream->fd = stream->ops->open(path, flags)))
    goto err_1;

  stream->rwflush = &no_flush;
  stream->pos = stream->ops->lseek(stream->fd, 0, SEEK_CUR);
  stream->buf_mode = _IOFBF;
  stream_fifo_init(&stream->fifo_read);
  stream_fifo_init(&stream->fifo_write);

  return (stream);

 err_1:
  free(stream);
 err:
  return NULL;
}

void set_buf_mode(FILE *stream, enum stdio_buf_mode_e mode)
{
  stream->rwflush(stream);
  stream->buf_mode = mode;
}

/* FIXME c'est nimp */
bool_t ferror(FILE *stream)
{
  return 0;
}

bool_t feof(FILE *stream)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  ssize_t	res;

  if (!stream_fifo_isempty(&stream->fifo_read))
    return 0;

  res = stream->ops->read(stream->fd, local, sizeof(local));

  if (res > 0)
    stream_fifo_pushback_array(&stream->fifo_read, local, res);

  return res <= 0;
}

