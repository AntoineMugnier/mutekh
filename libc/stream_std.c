
#include <stdio.h>

#include <hexo/types.h>

#include <hexo/device/char.h>
#include <hexo/device.h>
#include <hexo/driver.h>

/*

%config CONFIG_LIBC_STREAM_STD
desc Enable support for stdin, stdout and stderr through tty device
parent CONFIG_LIBC_STREAM
default defined
%config end

*/

extern struct device_s *tty_dev;

static ssize_t	tty_read(fd_t fd, void *buffer_, size_t count)
{
  uint8_t	*buffer = buffer_;
  ssize_t	res = 0;

  while (count > 0)
    {
      ssize_t	err;

      err = dev_char_read(tty_dev, buffer, count);

      if (err < 0)
	break;

      if (err > 0)
	{
	  count -= err;
	  buffer += err;
	  res += err;
	}
    }

  return res;  
}

static ssize_t	tty_write(fd_t fd, const void *buffer_, size_t count)
{
  const uint8_t	*buffer = buffer_;
  ssize_t	res = 0;

  while (count > 0)
    {
      ssize_t	err;

      err = dev_char_write(tty_dev, buffer, count);

      if (err < 0)
	break;

      if (err > 0)
	{
	  count -= err;
	  buffer += err;
	  res += err;
	}
    }

  return res;  
}

static error_t	empty_close(fd_t fd)
{
  return -1;
}

static off_t	empty_lseek(fd_t fd, off_t offset, enum stream_whence_e whence)
{
  return -1;
}

static bool_t	true_able(fd_t fd)
{
  return 1;
}

static bool_t	false_able(fd_t fd)
{
  return 0;
}

/****************************************** stdin */

static const struct stream_ops_s stdin_ops =
{
  .read = &tty_read,
  .close = &empty_close,
  .lseek = &empty_lseek,
  .readable = true_able,
  .writable = false_able,
};

static uint8_t stdin_buffer[CONFIG_LIBC_STREAM_BUFFER_SIZE];

static struct file_s stdin_file =
{
  .ops = &stdin_ops,
  .length = 0,
  .buffer = stdin_buffer,
  .last = STREAM_LAST_NONE,
};

FILE * const stdin = &stdin_file;

/****************************************** stdout */

static const struct stream_ops_s stdout_ops =
{
  .write = &tty_write,
  .close = &empty_close,
  .lseek = &empty_lseek,
  .readable = false_able,
  .writable = true_able,
};

static uint8_t stdout_buffer[CONFIG_LIBC_STREAM_BUFFER_SIZE];

static struct file_s stdout_file =
{
  .ops = &stdout_ops,
  .length = 0,
  .buffer = stdout_buffer,
  .last = STREAM_LAST_NONE,
};

FILE * const stdout = &stdout_file;
FILE * const stderr = &stdout_file;

