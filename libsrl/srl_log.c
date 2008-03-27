/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2008
 */

#include <stdio.h>

#include <hexo/types.h>

#include <hexo/device/char.h>
#include <drivers/device/char/tty-soclib/tty-soclib.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#ifdef CONFIG_DRIVER_CHAR_SOCLIBTTY

static ssize_t	tty_read(fd_t fd, void *buffer, size_t count)
{
	return dev_char_read((struct device_s *)fd, buffer, count);
}

static ssize_t	tty_write(fd_t fd, const void *buffer, size_t count)
{
  return dev_char_write((struct device_s *)fd, buffer, count);
}

static ssize_t	empty_io(fd_t fd, const void *buffer, size_t count)
{
	return count;
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

static error_t	no_flush(FILE *stream)
{
  return 0;
}

static const struct stream_ops_s tty_ops =
{
  .read = &tty_read,
  .write = &tty_write,
  .close = &empty_close,
  .lseek = &empty_lseek,
  .readable = true_able,
  .writable = true_able,
};

static const struct stream_ops_s blackhole_ops =
{
  .read = &empty_io,
  .write = &empty_io,
  .close = &empty_close,
  .lseek = &empty_lseek,
  .readable = true_able,
  .writable = true_able,
};

struct tty_state {
	uint8_t stdin_buffer[CONFIG_LIBC_STREAM_BUFFER_SIZE];
	struct file_s file;
	struct device_s device;
};

static struct file_s null_file =
{
	.ops = &blackhole_ops,
	.rwflush = &no_flush,
};

#include "soclib/tty.h"

struct device_s *tty_dev;
static struct tty_state _state[CONFIG_SRL_NTTY];

extern __ldscript_symbol_t _dsx_tty_address;
extern __ldscript_symbol_t _dsx_tty_no;

CPU_LOCAL FILE *cpu_tty;
CONTEXT_LOCAL FILE *context_tty;
FILE *tcg_tty;

static FILE *init_tty(void *addr)
{
	size_t i;
	if ( !addr )
		return &null_file;

	for ( i=0; i<CONFIG_SRL_NTTY; ++i ) {
		struct tty_state *st = &_state[i];

		if ( st->device.addr[0] == addr )
			return &st->file;

		if ( st->device.addr[0] == 0 ) {
			device_init(&st->device);
			st->device.addr[0] = addr;
			st->device.irq = 1;
			tty_soclib_init(&st->device, NULL, NULL);
			st->file.ops = &tty_ops;
			st->file.rwflush = &no_flush;
			st->file.fd = &st->device;
			return &st->file;
		}
	}
	return &null_file;
}

void srl_console_init()
{
	size_t i;
	uint32_t *tty = (uint32_t*)&_dsx_tty_address;
	uint32_t *addr = tty+TTY_SPAN*((size_t)&_dsx_tty_no);

	memset( &_state[0], 0, sizeof(_state[0])*CONFIG_SRL_NTTY );

	tcg_tty = NULL;
	assert(addr);
	tcg_tty = init_tty(addr);

	tty_dev = &_state[0].device;
}

void srl_console_init_cpu(void *addr)
{
	CPU_LOCAL_SET(cpu_tty, init_tty(addr));
}

void srl_console_init_task(void *addr)
{
	CONTEXT_LOCAL_SET(context_tty, init_tty(addr));
}

#else

void srl_console_init()
{
}

void srl_console_init_cpu(void *addr)
{
}

void srl_console_init_task(void *addr)
{
}

#endif
