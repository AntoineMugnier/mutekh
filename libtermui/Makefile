
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs = src/term.o src/term_none.o src/term_printf.o src/term_vt100.o	\
       src/term_vt102.o src/term_xterm.o src/strings.o termio_dev.o \

ifeq ($(CONFIG_LIBTERMUI_GETLINE), defined)
objs += src/getline.o src/getline_complete.o src/getline_edit.o	\
	src/getline_history.o
endif

ifeq ($(CONFIG_LIBTERMUI_CONSOLE), defined)
objs += src/console_builtin.o src/console.o src/console_process.o \
	src/console_entries.o src/console_parse.o \
        src/console_opt_integer.o src/console_opt_enum.o
ifeq ($(CONFIG_LIBTERMUI_CON_COMPLETION), defined)
objs += src/console_complete.o
endif
endif


DIR_CFLAGS=-I $(LOCAL_SRC_DIR)

