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

#include <device/char.h>
#include <drivers/device/char/tty-soclib/tty-soclib.h>
#include <hexo/device.h>
#include <device/driver.h>

#ifdef CONFIG_MUTEK_CONSOLE

#ifdef CONFIG_LIBC_STREAM

#include <fileops.h>

static FILEOPS_READ(tty_read)
{
	return dev_char_spin_read((struct device_s *)file, buffer, count);
}

static FILEOPS_WRITE(tty_write)
{
  return dev_char_spin_write((struct device_s *)file, buffer, count);
}

static FILEOPS_READ(empty_read)
{
	return count;
}

static FILEOPS_WRITE(empty_write)
{
	return count;
}

static FILEOPS_CLOSE(empty_close)
{
  return -1;
}

static FILEOPS_LSEEK(empty_lseek)
{
  return -1;
}

static error_t	no_flush(FILE *stream)
{
  return 0;
}

static const struct fileops_s tty_ops =
{
  .read = &tty_read,
  .write = &tty_write,
  .close = &empty_close,
  .lseek = &empty_lseek,
};

static const struct fileops_s blackhole_ops =
{
  .read = &empty_read,
  .write = &empty_write,
  .close = &empty_close,
  .lseek = &empty_lseek,
};

struct tty_state {
	struct file_s file;
	struct device_s device;
};

static struct file_s null_file =
{
	.ops = &blackhole_ops,
	.rwflush = &no_flush,
};

struct device_s *tty_dev;
static struct tty_state _state[CONFIG_SRL_NTTY];

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
			st->device.irq = 0;
			st->device.icudev = NULL;
			tty_soclib_init(&st->device, NULL);
			__stdio_stream_init(&st->file);
			st->file.ops = &tty_ops;
			st->file.hndl = &st->device;
			return &st->file;
		}
	}
	return &null_file;
}

void srl_console_init(void *addr)
{
	__stdio_stream_init(&null_file);

	memset( &_state[0], 0, sizeof(_state[0])*CONFIG_SRL_NTTY );

	tcg_tty = NULL;
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

#else // not CONFIG_LIBC_STREAM

static struct device_s device;
struct device_s *tty_dev;

void srl_console_init(void *addr)
{
	device_init(&device);
	device.addr[0] = addr;
	device.irq = 1;
	tty_soclib_init(&device, NULL, NULL);
	tty_dev = &device;
}

void srl_console_init_cpu(void *addr)
{
}

void srl_console_init_task(void *addr)
{
}

#endif // end CONFIG_LIBC_STREAM

#else // not CONFIG_MUTEK_CONSOLE

void srl_console_init(void *addr)
{
}

void srl_console_init_cpu(void *addr)
{
}

void srl_console_init_task(void *addr)
{
}

#endif
