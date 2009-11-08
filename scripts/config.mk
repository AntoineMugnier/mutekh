
CONF_PATH:=$(MUTEK_SRC_DIR):$(CURRENT_DIR):$(USER_DIR)
CONF_TMP_BASE:=$(shell mktemp /tmp/mutekh_config.XXXXXX)
CONF_EXTS = py m4 h mk deps

CONFIG_TMP_FILES=$(addprefix $(CONF_TMP_BASE),$(CONF_EXTS))
CONFIG_FILES=$(addprefix $(CONF_DIR)/.config.,$(CONF_EXTS))

$(CONF_DIR):
	mkdir -p $@

ifeq (0,1)
define do_conf

$(CONF_DIR)/.config.$(1): $$(CONF_TMP_BASE).$(1)
	diff -q $$@ $$< || cp $$< $$@

endef

$(eval $(foreach e,$(CONF_EXTS),$(call do_conf,$(e))))
endif

ifeq (0,1)

$(CONF_DIR)/.config.%: $(CONF_TMP_BASE).%
	@if [ -r "$@" ] ; then \
		if diff -q $@ $< 2>/dev/null ; then \
			echo "  CONF OK   $@" ; \
		else \
			echo "  CONF RE   $@" ; \
			cp $< $@ ; \
		fi ; \
	else \
		echo "  CONF      $@" ; \
		cp $< $@ ; \
	fi

$(CONFIG_TMP_FILES): $(CONF)
	@test -d $(CONF_DIR) || mkdir -p $(CONF_DIR)
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF)					\
		--python=$(CONF_TMP_BASE).py		\
		--m4=$(CONF_TMP_BASE).m4			\
		--header=$(CONF_TMP_BASE).h			\
		--depmakefile=$(CONF_TMP_BASE).deps			\
		--makefile=$(CONF_TMP_BASE).mk

else

$(CONFIG_FILES): $(CONF) $(MUTEK_SRC_DIR)/scripts/config.pl
	@test -d $(CONF_DIR) || mkdir -p $(CONF_DIR)
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF)					\
		--python=$(CONF_DIR)/.config.py		\
		--m4=$(CONF_DIR)/.config.m4			\
		--header=$(CONF_DIR)/.config.h			\
		--depmakefile=$(CONF_DIR)/.config.deps			\
		--makefile=$(CONF_DIR)/.config.mk

endif

config: $(CONF_DIR) $(CONFIG_FILES)

checkconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF) --check

couple:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF) --arch-cpu

listconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF) --list

listallconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF) --list=all

showconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
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
