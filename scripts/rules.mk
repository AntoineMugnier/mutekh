export SRC_DIR
export BUILD_DIR

include $(SRC_DIR)/config.mk

ifneq ($(H),/)
include $(SRC_DIR)$(H)/Makefile
endif
-include depend.mk

VPATH = $(SRC_DIR)$(H)

deps = $(patsubst %.o,.depends/%.deps,$(objs))

include $(SRC_DIR)/scripts/common.mk

depend.mk: $(deps)
	@cat /dev/null $^ > $@

.depends/%.deps: %.S
	@test -d .depends || mkdir .depends
	@-$(CC) $(CFLAGS) $(INCS) -MM -MF $@ $<

.depends/%.deps: %.c
	@test -d .depends || mkdir .depends
	@-$(CC) $(CFLAGS) $(INCS) -MM -MF $@ $<


