export SRC_DIR
export BUILD_DIR

include $(SRC_DIR)/arch/current/config.mk
include $(SRC_DIR)/cpu/current/config.mk

ifneq ($(H),/)
include $(SRC_DIR)$(H)/Makefile
endif
-include depend.mk

VPATH = $(SRC_DIR)$(H)

deps = $(patsubst %.o,.depends/%.deps,$(objs))

depend.mk: $(deps)
	@cat /dev/null $^ > $@

.depends/%.deps: %.S
	@test -d .depends || mkdir .depends
	@-$(CC) $(CFLAGS) $(INCS) -MM -MF $@ $<

.depends/%.deps: %.c
	@test -d .depends || mkdir .depends
	@-$(CC) $(CFLAGS) $(INCS) -MM -MF $@ $<

include $(SRC_DIR)/scripts/common.mk

