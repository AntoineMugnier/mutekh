
include $(BUILD_DIR)/config.mk

default: $(BUILD_DIR)/arch/current $(BUILD_DIR)/cpu/current

$(BUILD_DIR)/arch/current::
	ln -sfn $(CONFIG_ARCH_NAME) $(BUILD_DIR)/arch/current

$(BUILD_DIR)/cpu/current::
	ln -sfn $(CONFIG_CPU_NAME) $(BUILD_DIR)/cpu/current

