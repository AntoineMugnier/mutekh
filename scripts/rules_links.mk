
include $(BUILD_DIR)/config.mk

default: $(BUILD_DIR)/arch/current $(BUILD_DIR)/cpu/current
	mkdir -p $(BUILD_DIR)/include
	ln -sfn $(BUILD_DIR)/arch/current/include $(BUILD_DIR)/include/arch
	ln -sfn $(BUILD_DIR)/cpu/current/include $(BUILD_DIR)/include/cpu

$(BUILD_DIR)/arch/current::
	mkdir -p $(BUILD_DIR)/arch
	ln -sfn $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME) $(BUILD_DIR)/arch/current

$(BUILD_DIR)/cpu/current::
	mkdir -p $(BUILD_DIR)/cpu
	ln -sfn $(SRC_DIR)/cpu/$(CONFIG_CPU_NAME) $(BUILD_DIR)/cpu/current

