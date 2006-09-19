BASE_PWD:=$(PWD)
BUILD_DIR:=$(BASE_PWD)
SRC_DIR=$(BASE_PWD)

export SRC_DIR
export BUILD_DIR

include $(SRC_DIR)/arch/current/config.mk
include $(SRC_DIR)/cpu/current/config.mk

target = kernel-$(ARCH)-$(CPU).out

subdirs = arch \
cpu \
drivers \
hexo \
libc \
libpthread \
libnetwork
objs = 

include $(SRC_DIR)/scripts/common.mk

.PRECIOUS: $(target)

$(target): $(objs) $(subdirs-lists) $(SRC_DIR)/arch/$(ARCH)/ldscript $(LIBAPP)
	@echo '  LD   $@'
	@$(LD) -q $$(cat /dev/null $(filter %.list,$^)) \
	$(filter %.o,$^) $(filter %.a,$^) \
	-T $(SRC_DIR)/arch/$(ARCH)/ldscript \
	-o $@
