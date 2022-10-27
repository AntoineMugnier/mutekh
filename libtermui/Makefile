
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs = src/term.o src/term_none.o src/term_printf.o src/term_vt100.o	\
       src/term_vt102.o src/term_xterm.o src/strings.o termio_dev.o \

ifeq ($(CONFIG_LIBTERMUI_GETLINE), defined)
objs += src/getline.o src/getline_edit.o	\
	src/getline_history.o
endif

ifeq ($(CONFIG_LIBTERMUI_CONSOLE), defined)
objs += src/console_builtin.o src/console.o src/console_process.o \
	src/console_parse.o src/console_opt.o
ifeq ($(CONFIG_LIBTERMUI_CON_COMPLETION), defined)
objs += src/console_complete.o
endif
endif

ext_hg = libtermui d23a52778ae5a6bd85bfb61cfb3d535f2b66f67f http://hg.savannah.nongnu.org/hgweb/libtermui

DIR_CFLAGS=-I $(LOCAL_SRC_DIR)

