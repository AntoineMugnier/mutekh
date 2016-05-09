MKDOC=mkdoc

include $(MUTEK_SRC_DIR)/doc/header_list.mk

include $(MUTEK_SRC_DIR)/scripts/discover.mk

doc:
	mkdir -p $(BUILD_DIR)/doc
	perl $(MUTEK_SRC_DIR)/scripts/config.pl --src-path=$(MUTEK_SRC_DIR) --docheader=$(BUILD_DIR)/doc/config.h
	cd $(MUTEK_SRC_DIR) ; \
	$(MKDOC) $(MKDOCFLAGS) \
	  --mkdoclib-create mutek-api \
	  --mkdoclib-url http://www.mutekh.org/www/mutekh_api/ \
	  --output-path $(BUILD_DIR)/doc \
	  -I $(BUILD_DIR) -I . \
	  -I doc/include \
	  $(subst $(MUTEK_SRC_DIR)/,,$(MKDOC_ARGS))

.PHONY: doc
