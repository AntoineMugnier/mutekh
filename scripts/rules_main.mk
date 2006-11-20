
LDFLAGS=

-include $(BUILD_DIR)/arch/current/config.mk
-include $(BUILD_DIR)/cpu/current/config.mk
-include $(BUILD_DIR)/.config.mk

target = kernel-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME).out

subdirs = arch cpu drivers hexo mutek libc

ifeq ($(CONFIG_NETWORK), defined)
 subdirs += libnetwork
endif

ifeq ($(CONFIG_PTHREAD), defined)
 subdirs += libpthread
endif

objs =

include $(SRC_DIR)/scripts/common.mk

kernel: $(target)

$(target): $(BUILD_DIR)/.config.h $(objs) $(subdirs-lists) $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/ldscript $(LIBAPP)
	echo $$'\n ------------------------'
	echo '    LD      $@'
	$(LD) $(LDFLAGS) $(ARCHLDFLAGS) \
		-q $$(cat /dev/null $(filter %.list,$^)) \
		$(filter %.o,$^) $(filter %.a,$^) \
		-T $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/ldscript \
		-o $(BUILD_DIR)/$@

clean:
	rm -f $(BUILD_DIR)/$(target)

