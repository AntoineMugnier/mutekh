
CONF_TMP_BASE:=$(shell mktemp)
CONF_EXTS = py m4 h mk

$(CONF_DIR):
	mkdir -p $@

ifeq (0,1)
define do_conf

$(CONF_DIR)/.config.$(1): $$(CONF_TMP_BASE).$(1)
	diff -q $$@ $$< || cp $$< $$@
	rm -f $$<

endef

$(eval $(foreach e,$(CONF_EXTS),$(call do_conf,$(e))))
endif


$(CONF_DIR)/.config.%: $(CONF_TMP_BASE).%
	(diff -q $@ $< || cp $< $@ ) 2>&1 > /dev/null
	rm -f $<


$(CONF_TMP_BASE).m4 $(CONF_TMP_BASE).mk $(CONF_TMP_BASE).py $(CONF_TMP_BASE).h: $(CONF)
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(MUTEK_SRC_DIR):$(CURRENT_DIR) \
		--input=$(CONF)					\
		--python=$(CONF_TMP_BASE).py		\
		--m4=$(CONF_TMP_BASE).m4			\
		--header=$(CONF_TMP_BASE).h			\
		--makefile=$(CONF_TMP_BASE).mk

config: $(CONF_DIR) $(CONF_DIR)/.config.mk $(CONF_DIR)/.config.py $(CONF_DIR)/.config.m4 $(CONF_DIR)/.config.h

checkconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(MUTEK_SRC_DIR):$(CURRENT_DIR) \
		--input=$(CONF) --check

listconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(MUTEK_SRC_DIR):$(CURRENT_DIR) \
		--input=$(CONF) --list

listallconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(MUTEK_SRC_DIR):$(CURRENT_DIR) \
		--input=$(CONF) --list=all

showconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(MUTEK_SRC_DIR):$(CURRENT_DIR) \
		--input=$(CONF) --info=$(TOKEN)

$(CONF):
	test -f $(CONF) || ( $(MAKE) helpconfig -f $(MUTEK_SRC_DIR)/Makefile ; false )

helpconfig:
	echo "The \`$(CONF)' source configuration file is missing."
	echo "Please set the CONF variable to use an alternative"
	echo "file or provide the missing file.\n"
	echo "Available configuration options can be displayed"
	echo "with 'make listconfig' and 'make listallconfig'.\n"
	echo "Informations about a specific configuration token"
	echo "can be displayed with 'make showconfig TOKEN=...'.\n"
