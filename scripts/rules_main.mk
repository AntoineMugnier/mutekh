
LDFLAGS=
TARGET_EXT=out

-include $(BUILD_DIR)/arch/current/config.mk
-include $(BUILD_DIR)/cpu/current/config.mk
-include $(BUILD_DIR)/.config.mk

target = kernel-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)

subdirs = arch cpu drivers hexo mutek libc

ifeq ($(CONFIG_NETWORK), defined)
 subdirs += libnetwork
endif

ifeq ($(CONFIG_PTHREAD), defined)
 subdirs += libpthread
endif

objs =

include $(SRC_DIR)/scripts/common.mk

kernel: $(target).$(TARGET_EXT)

$(target).out: $(BUILD_DIR)/.config.h $(objs) $(subdirs-lists) arch/$(CONFIG_ARCH_NAME)/ldscript $(LIBAPP)
	echo '    LD      $@'
	$(LD) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $$(cat /dev/null $(filter %.list,$^)) \
		$(filter %.o,$^) $(filter %.a,$^) \
		-T $(BUILD_DIR)/arch/$(CONFIG_ARCH_NAME)/ldscript \
		-o $(BUILD_DIR)/$@

$(target).hex: $(target).out
	echo 'OBJCOPY HEX $@'
	$(OBJCOPY) -j .text -j .data -j .boot -j .contextdata -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(target).bin: $(target).out
	echo 'OBJCOPY HEX $@'
	$(OBJCOPY) -j .text -j .data -j .boot -j .contextdata -O binary $(BUILD_DIR)/$< $(BUILD_DIR)/$@

clean:
	rm -f $(BUILD_DIR)/$(target)

