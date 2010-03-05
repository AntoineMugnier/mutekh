objs =	\
term.o term_none.o term_printf.o	\
term_telnet.o term_vt100.o term_vt102.o \
term_xterm.o

ifeq ($(CONFIG_LIBTERMUI_GETLINE), defined)
objs += getline.o getline_complete.o	\
	getline_edit.o getline_history.o
endif

ifeq ($(CONFIG_LIBTERMUI_CONSOLE), defined)
objs += console_builtin.o console.o console_complete.o	\
	console_entries.o console_parse.o		\
	console_process.o
endif

