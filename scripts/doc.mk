include $(MUTEK_SRC_DIR)/doc/header_list.mk

include $(MUTEK_SRC_DIR)/scripts/discover.mk

$(BUILD_DIR)/doc/config.h:
	test -d $(BUILD_DIR)/doc || mkdir -p $(BUILD_DIR)/doc
	$(MUTEK_SRC_DIR)/scripts/config.pl --docheader=$@

doc: $(BUILD_DIR)/doc/config.h
	cd $(MUTEK_SRC_DIR) ; \
	mkdoc $(MKDOCFLAGS) doc/gpct.mkdoclib \
	  --output-path $(BUILD_DIR)/doc \
	  -I $(BUILD_DIR) doc/config.h \
	  -I . \
	  -I doc/include \
	  $(MKDOC_ARGS) \
	  $(CPU_HEADER) $(ARCH_HEADER)

.PHONY: doc
