
#include <unistd.h>
#include <fileops.h>
#include <stdio.h>

#include <hexo/types.h>

#include <device/char.h>
#include <device/device.h>
#include <device/driver.h>

#if defined(CONFIG_MUTEK_CONSOLE)
extern struct device_s *console_dev;
#endif

static FILEOPS_READ(tty_read)
{
  return dev_char_wait_read(console_dev, buffer, count);
}

static FILEOPS_WRITE(tty_write)
{
  return dev_char_wait_write(console_dev, buffer, count);
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


fd_t fd_add(const struct fileops_s *ops, void *hndl);

static inline void _fd_add(const struct fileops_s *ops, void *hndl)
{
	fd_t fd = fd_add(ops, hndl);

//	printk("New fd: %p, %d\n", hndl, fd);
}

void stdio_in_out_err_init()
{
	fd_t fd;
	fd = fd_add(&stdin_ops, stdin);
	assert(fd == 0);
	fd = fd_add(&stdout_ops, stdout);
	assert(fd == 1);
	fd = fd_add(&stdout_ops, stderr);
	assert(fd == 2);
}
