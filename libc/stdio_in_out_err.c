
#include <unistd.h>
#include <fileops.h>
#include <stdio.h>

#include <hexo/types.h>

#include <device/char.h>
#include <hexo/device.h>
#include <device/driver.h>

#if defined(CONFIG_MUTEK_CONSOLE)
extern struct device_s *tty_dev;
#endif

static FILEOPS_READ(tty_read)
{
  return dev_char_wait_read(tty_dev, buffer, count);
}

static FILEOPS_WRITE(tty_write)
{
  return dev_char_wait_write(tty_dev, buffer, count);
}

static error_t	no_flush(FILE *stream)
{
  return 0;
}

/****************************************** stdin */

static const struct fileops_s stdin_ops =
{
  .read = &tty_read,
};

static struct file_s stdin_file =
{
  .ops = &stdin_ops,
  .buf_mode = _IOLBF,
  .rwflush = &no_flush,
};

FILE * const stdin = &stdin_file;

/****************************************** stdout */

static const struct fileops_s stdout_ops =
{
  .write = &tty_write,
};

static struct file_s stdout_file =
{
  .ops = &stdout_ops,
  .buf_mode = _IOLBF,
  .rwflush = &no_flush,
};

FILE * const stdout = &stdout_file;

/****************************************** stderr */

static struct file_s stderr_file =
{
  .ops = &stdout_ops,
  .buf_mode = _IONBF,
  .rwflush = &no_flush,
};

FILE * const stderr = &stderr_file;

