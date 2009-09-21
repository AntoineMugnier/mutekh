
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <fileops.h>

#ifdef CONFIG_VFS
#include <vfs/vfs.h>
#endif

/***********************************************************************
            Internal buffered stream API
***********************************************************************/

CONTAINER_FUNC(stream_fifo, RING, static inline, stream_fifo);

static error_t	__stdio_no_flush(FILE *stream)
{
  return 0;
}

static error_t	__stdio_write_flush(FILE *stream)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  uint8_t	*ptr = local;
  size_t	size = stream_fifo_pop_array(&stream->fifo_write, ptr, CONFIG_LIBC_STREAM_BUFFER_SIZE);
  ssize_t	res;

  /* write remaining data present in buffer */
  while (size)
    {
      res = stream->ops->write(stream->hndl, ptr, size);

      if (res < 0)
	{
	  stream->error = 1;
	  stream_fifo_pushback_array(&stream->fifo_write, ptr, size);
	  return res;
	}

      size -= res;
      ptr += res;
    };

  /* write buffer is empty here */
  stream->rwflush = &__stdio_no_flush;
  return 0;
}

static error_t	__stdio_read_flush(FILE *stream)
{
  /* seek fd to real pos and flush prefetched data in read buffer */
  stream->ops->lseek(stream->hndl, stream->pos, SEEK_SET);
  stream_fifo_clear(&stream->fifo_read);

  /* read buffer is empty here */
  stream->rwflush = &__stdio_no_flush;
  return 0;
}

error_t	__stdio_read(size_t size, FILE *stream, uint8_t *ptr)
{
  uint8_t	local[CONFIG_LIBC_STREAM_BUFFER_SIZE];
  ssize_t	res, local_size;

  if (!stream->ops->read)
    return -EINVAL;

  /* get data from buffer */
  res = stream_fifo_pop_array(&stream->fifo_read, ptr, size);
  size -= res;
  ptr += res;
  stream->pos += res;

  if (size)
    {
      /* read buffer is empty here */
      stream->rwflush = &__stdio_no_flush;

      /* read more data directly from fd */
      while (size > CONFIG_LIBC_STREAM_BUFFER_SIZE)
	{
	  res = stream->ops->read(stream->hndl, ptr, CONFIG_LIBC_STREAM_BUFFER_SIZE);

	  if (res <= 0)
	    {
	      if (res == 0)
		stream->eof = 1;
 	      else
		stream->error = 1;

	      return res;
	    }

	  stream->eof = 0;
	  size -= res;
	  ptr += res;
	  stream->pos += res;
	}
    }

  /* read remaining data in local buffer */
  for (local_size = 0; local_size < size; local_size += res)
    {
      res = stream->ops->read(stream->hndl, local + local_size,
			      CONFIG_LIBC_STREAM_BUFFER_SIZE - local_size);

      if (res < 0)
	{
	  stream->error = 1;
	  return res;
	}
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
	  stream->rwflush = &__stdio_read_flush;
	}
      return 1;
    }
  else
    {
      /* not enough data have been read */
      stream->eof = 1;
      return 0;
    }
}

static error_t unbuffered_write(size_t size, FILE *stream, const uint8_t *ptr)
{
  ssize_t res;

  while (size)
    {
      res = stream->ops->write(stream->hndl, ptr, size);

      if (res < 0)
	{
	  stream->error = 1;
	  return res;
	}

      stream->pos += res;
      size -= res;
      ptr += res;
    }

  return 0;
}

error_t __stdio_write(size_t size, FILE *stream, const uint8_t *ptr)
{
  if (!stream->ops->write)
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

	    if ((res = __stdio_write_flush(stream)))
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
	  if ((res = __stdio_write_flush(stream)))
	    return res;

	  /* write data directly to device if greater than buffer */
	  while (size > CONFIG_LIBC_STREAM_BUFFER_SIZE)
	    {
	      res = stream->ops->write(stream->hndl, ptr, size);

	      if (res < 0)
		{
		  stream->error = 1;
		  return res;
		}

	      size -= res;
	      ptr += res;
	      stream->pos += res;
	    }
	}

      /* fill buffer with remaining data */
      stream_fifo_pushback_array(&stream->fifo_write, (uint8_t*)ptr, size);
      stream->pos += size;
      stream->rwflush = &__stdio_write_flush;
    }

  return 0;
}

/***********************************************************************
            Stdio functions
***********************************************************************/

error_t	fclose(FILE *stream)
{
  error_t	err;

  if (!stream->ops->close)
    return -EINVAL;

  if ((err = stream->rwflush(stream)))
    return err;

  if ((err = stream->ops->close(stream->hndl)))
    return err;

  free(stream);

  return (err);
}

error_t	fflush(FILE *stream)
{
  if (!stream->ops->write)
    return -EINVAL;

  return __stdio_write_flush(stream);
}

error_t	fpurge(FILE *stream)
{
  if (!stream->ops->read)
    return -EINVAL;

  return __stdio_read_flush(stream);
}

/* ************************************************** */

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

/* ************************************************** */

error_t fseek(FILE *stream, fpos_t offset, int_fast8_t whence)
{
  if (!stream->ops->lseek)
    return EOF;

  stream->rwflush(stream);

  if ((stream->pos = stream->ops->lseek(stream->hndl, offset, whence)) >= 0)
    return 0;
  else
    return EOF;
}

/* ************************************************** */

int_fast16_t fgetc(FILE *stream)
{
  unsigned char	res;
  return __stdio_read(1, stream, &res) <= 0 ? EOF : res;
}

int_fast16_t ungetc(int_fast16_t c, FILE *stream)
{
  return stream_fifo_push(&stream->fifo_read, c) ? c : EOF;
}

char *fgets(char *str_, size_t size, FILE *stream)
{
  char	*str = str_;
  char	*ret = NULL;

  while (size-- > 1)
    {
	  error_t res = __stdio_read(1, stream, (uint8_t*)str);

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

/* ************************************************** */

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

/* ************************************************** */

error_t setvbuf(FILE *stream, char *buf, enum stdio_buf_mode_e mode, size_t size)
{
  stream->rwflush(stream);
  stream->buf_mode = mode;
  return 0;
}

/* ************************************************** */

void __stdio_stream_init(FILE *file)
{
  file->rwflush = &__stdio_no_flush;
  stream_fifo_init(&file->fifo_read);
  stream_fifo_init(&file->fifo_write);

  file->pos = 0;
  file->buf_mode = _IONBF;
  file->error = 0;
  file->eof = 0;
}

#ifdef CONFIG_VFS

static vfs_open_flags_t	open_flags(const char *str)
{
  vfs_open_flags_t	flags = 0;

  while (*str)
    {
      switch (*str)
	{
	case ('r'):
	  flags |= VFS_O_RDONLY;
	  break;

	case ('w'):
	  flags |= VFS_O_WRONLY | VFS_O_CREATE;
	  break;

	case ('+'):
	  flags |= VFS_O_RDWR;
	  break;

	case ('a'):
	  flags |= O_WRONLY | VFS_O_APPEND;
	  break;

	case ('b'):
	  break;

	default:
	  return -1;
	}
      str++;
    }

  return (flags);
}

static const struct fileops_s fopen_fops =
{
  .read = (fileops_read_t*)vfs_read,
  .write = (fileops_write_t*)vfs_write,
  .lseek = (fileops_lseek_t*)vfs_lseek,
  .close = (fileops_close_t*)vfs_close,
};

FILE *fopen(const char *path, const char *mode)
{
  FILE		*file;
  uint_fast8_t	flags;

  if ((flags = open_flags(mode)) < 0)
    goto err;

  if (!(file = malloc(sizeof (FILE))))
    goto err;

  file->ops = &fopen_fops;

  if (vfs_open(vfs_get_root(), path, flags, 0644, &file->hndl))
    goto err_1;

  __stdio_stream_init(file);

  file->pos = vfs_lseek(file->hndl, 0, SEEK_CUR);
  file->buf_mode = _IOFBF;

  return (file);

 err_1:
  free(file);
 err:
  return NULL;
}

#endif /* CONFIG_VFS */

