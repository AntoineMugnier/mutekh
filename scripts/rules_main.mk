
LDFLAGS=
TARGET_EXT=out
TARGET_SECTIONS=.text .data .boot .contextdata

-include $(BUILD_DIR)/.config.mk

target = kernel-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)

MODULES += libc

ifeq ($(CONFIG_NETWORK), defined)
 MODULES += libnetwork
endif

ifeq ($(CONFIG_PTHREAD), defined)
 MODULES += libpthread
endif

subdirs = $(MODULES) arch cpu drivers hexo mutek

objs =

-include $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/config.mk
-include $(SRC_DIR)/cpu/$(CONFIG_CPU_NAME)/config.mk
include $(SRC_DIR)/scripts/common.mk

kernel: $(target).$(TARGET_EXT)

$(target).out: $(BUILD_DIR)/.config.h $(objs) $(subdirs-lists) \
		arch/$(CONFIG_ARCH_NAME)/ldscript \
		cpu/$(CONFIG_CPU_NAME)/ldscript $(LIBAPP)
	echo '    LD      $@'
	$(LD) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $$(cat /dev/null $(filter %.list,$^)) \
		$(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $(BUILD_DIR)/$@

$(target).hex: $(target).out
	echo 'OBJCOPY HEX $@'
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(target).bin: $(target).out
	echo 'OBJCOPY BIN $@'
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O binary $(BUILD_DIR)/$< $(BUILD_DIR)/$@

clean:
	rm -f $(BUILD_DIR)/$(target)

