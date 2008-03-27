/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef STDIO_H_
#define STDIO_H_

#include <hexo/types.h>
#include <hexo/error.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_ring.h>

#include <stdarg.h>

int_fast8_t putchar(char c);
int_fast8_t puts(const char *s);
void __puts(const char *s, size_t len);

int_fast8_t getchar(void);
char *gets(char *s);

ssize_t printf(const char *format, ...);
ssize_t sprintf(char *str, const char *format, ...);
ssize_t snprintf(char *str, size_t size, const char *format, ...);

ssize_t vprintf(const char *format, va_list ap);
ssize_t vsprintf(char *str, const char *format, va_list ap);
ssize_t vsnprintf(char *str, size_t size, const char *format, va_list ap);

ssize_t scanf(const char *format, ...);
ssize_t sscanf(const char *str, const char *format, ...);
ssize_t vscanf(const char *format, va_list ap);
ssize_t vsscanf(const char *str, const char *format, va_list ap);

#ifdef CONFIG_LIBC_STREAM

#define O_RDONLY	0x01
#define O_WRONLY	0x02
#define O_RDWR		0x03
#define O_CREAT		0x10
#define O_TRUNC		0x20
#define O_APPEND	0x40

enum stream_whence_e
  {
    SEEK_SET, SEEK_END, SEEK_CUR
  };

CONTAINER_TYPE(stream_fifo, RING, uint8_t, CONFIG_LIBC_STREAM_BUFFER_SIZE);

# define EOF			-1

typedef void *			fd_t;
typedef int16_t			mode_t;
typedef int32_t			fpos_t;

struct				stream_ops_s
{
  fd_t (*open)(const char *name, uint_fast8_t mode);
  ssize_t (*write)(fd_t fd, const void *buffer, size_t count);
  ssize_t (*read)(fd_t fd, void *buffer, size_t count);
  error_t (*close)(fd_t fd);
  off_t	(*lseek)(fd_t fd, off_t offset, enum stream_whence_e whence);
  bool_t (*readable)(fd_t fd);
  bool_t (*writable)(fd_t fd);
};

extern const struct stream_ops_s	*stream_fops;

struct				file_s
{
  const struct stream_ops_s	*ops;
  fd_t				fd;
  stream_fifo_root_t		fifo_read;
  stream_fifo_root_t		fifo_write;
  fpos_t			pos;
  error_t			(*rwflush)(struct file_s *stream);
};

typedef struct file_s		FILE;

extern FILE * const stdin;
extern FILE * const stdout;
extern FILE * const stderr;

FILE *fopen(const char *path, const char *mode);
error_t fclose(FILE *stream);
uint_fast16_t fputc(unsigned char c, FILE *stream);
uint_fast16_t fgetc(FILE *stream);
size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
char *fgets(char *str, size_t size, FILE *stream);
error_t fputs(const char *str, FILE *stream);
error_t fseek(FILE *stream, fpos_t offset, int_fast8_t whence);
fpos_t ftell(FILE *stream);
void rewind(FILE *stream);
error_t fgetpos(FILE *stream, fpos_t *pos);
error_t fsetpos(FILE *stream, const fpos_t *pos);
error_t fflush(FILE *stream);
error_t fpurge(FILE *stream);
ssize_t vfprintf(FILE *stream, const char *format, va_list ap);
ssize_t fprintf(FILE *stream, const char *format, ...);
bool_t	feof(FILE *stream);

#endif /* CONFIG_LIBC_STREAM */

#endif

