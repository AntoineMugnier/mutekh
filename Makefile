
CC=gcc
CPP=cpp
LD=ld
AS=as

LIBAPP=libapp.a

#######################################################################

CFLAGS=-Wall -Winline \
	-O2 -fomit-frame-pointer \
	-mno-tls-direct-seg-refs \
	-fno-builtin

INCS=-nostdinc -D__MUTEK__ -Iinclude -include mutek/config.h

subdirs-y=	arch/current \
		cpu/current \
		drivers/tty-vga \
		drivers/fb-vga \
		drivers/uart-8250 \
		drivers/icu-8259 \
		main \
		libc \
		libpthread \

define recurs
  $(eval include $(1)/Makefile)
endef

$(foreach dir,$(subdirs-y),$(eval $(call recurs,$(dir))))

all: kernel.out

kernel.out: $(OBJ-y) arch/current/ldscript $(LIBAPP)
	@printf $$'LD      %s\n' "$@"
	@$(LD) -o $@ -T arch/current/ldscript $(OBJ-y) $(LIBAPP)

%.o: %.S
	@printf $$'AS      %s\n' "$@"
	@$(CPP) $(INCS) $< | $(AS) -o $@

%.o: %.c
	@printf $$'CC      %s\n' "$@"
	@$(CC) $(CFLAGS) $(INCS) -c $< -o $@

clean:
	@printf $$'CLEAN\n' "$@"
	@find . -name '*~' -exec rm '{}' ';'
	@rm -f $(OBJ-y) kernel.out

re: clean all

