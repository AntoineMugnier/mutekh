
default: arch_cpu_dirs $(target)

CFLAGS=-Wall \
	-O2 -fomit-frame-pointer \
	-fno-builtin

INCS=-nostdinc -D__TEST__ -D__MUTEK__ -D__ARCH__$(ARCH)__ -D__CPU__$(CPU)__ -I$(SRC_DIR)/include -include $(SRC_DIR)/config.h

%.o: %.S
	@echo '  AS   $@'
	@$(CPP) $(INCS) $< | $(AS) -o $@

%.o: %.c
	@echo '  CC   $@'
	@$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -c $< -o $@

subdirs-lists = $(foreach name,$(subdirs),$(patsubst %,%.list,$(name)))
CC=$(CPUTOOLS)gcc
CPP=$(CPUTOOLS)cpp
LD=$(CPUTOOLS)ld
AR=$(CPUTOOLS)ar
AS=$(CPUTOOLS)as
OBJCOPY=$(CPUTOOLS)objcopy
OBJDUMP=$(CPUTOOLS)objdump

MAKEFLAGS += -s

arch/$(ARCH):
	@mkdir -p $@

cpu/$(CPU):
	@mkdir -p $@

arch/current:
	@ln -s $(ARCH) $@

cpu/current:
	@ln -s $(CPU) $@

arch_cpu_dirs: arch/$(ARCH) cpu/$(CPU) arch/current cpu/current

clean:
	@echo "CLEAN $(H)"
	@rm -f *.out *~ depend.mk $(objs) $(subdirs-lists) *.map
	@rm -rf .depends
	@for i in $(subdirs) ; do \
		make -C $$i -f $(SRC_DIR)/scripts/rules.mk H="$(H)/$$i" clean; \
	done

../$(DIR).list: $(objs) $(subdirs-lists) $(SRC_DIR)$(H)/Makefile
	@echo " LIST $(H)"
	@rm -f $@
	@for obj in $(filter %.o,$^) \
			$$(cat /dev/null $(filter %.list,$^)) ; do \
		echo $(DIR)/$${obj} >> $@ ; \
	done

%.list: $(SRC_DIR)$(H)/%/Makefile
	@test -d $* || mkdir -p $*
	@$(MAKE) -C $* -f $(SRC_DIR)/scripts/rules.mk ../$@ DIR=$* H="$(H)/$*"
