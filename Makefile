
CC=$(CPUTOOLS)gcc
CPP=$(CPUTOOLS)cpp
LD=$(CPUTOOLS)ld
AS=$(CPUTOOLS)as

#LIBAPP=libapp.a

#######################################################################

CFLAGS=-Wall -Winline \
	-O2 -fomit-frame-pointer \
	-fno-builtin

INCS=-nostdinc -D__MUTEK__ -D__ARCH__$(ARCH)__ -D__CPU__$(CPU)__ -Iinclude -include mutek/config.h

subdirs-y=	arch/current \
		cpu/current \
		drivers \
		main \
		libc \
		libpthread \

define recurs
  $(eval include $(1)/Makefile)
endef

$(foreach dir,$(subdirs-y),$(eval $(call recurs,$(dir))))

KOUT=kernel-$(ARCH)-$(CPU).out

all: $(KOUT)

$(KOUT): $(OBJ-y) arch/current/ldscript $(LIBAPP)
	@printf $$'LD      %s\n' "$@"
	@$(LD) -o $@ -q -T arch/current/ldscript $(OBJ-y) $(LIBAPP)

%.o: %.S
	@printf $$'AS      %s\n' "$@"
	@$(CPP) $(INCS) $< | $(AS) -o $@

%.o: %.c
	@printf $$'CC      %s\n' "$@"
	@$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -c $< -o $@

clean:
	@printf $$'CLEAN\n' "$@"
	@find . -name '*~' -exec rm '{}' ';'
	@rm -f $(OBJ-y) $(KOUT)

re: clean all

