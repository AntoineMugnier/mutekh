
CONF_PATH:=$(MUTEK_SRC_DIR):$(CURRENT_DIR):$(USER_DIR)
CONF_TMP_BASE:=$(shell mktemp /tmp/mutekh_config.XXXXXX)
CONF_EXTS = py m4 h mk

CONFIG_TMP_FILES=$(addprefix $(CONF_TMP_BASE).,$(CONF_EXTS))
CONFIG_FILES=$(addprefix $(CONF_DIR)/.config.,$(CONF_EXTS) deps)

$(CONF_DIR):
	mkdir -p $@

$(CONF_DIR)/.config.%: $(CONF_TMP_BASE).%
	test -d $(CONF_DIR) || mkdir -p $(CONF_DIR)
	if [ -r "$@" ] ; then \
		if diff -q $@ $< 2>&1 > /dev/null ; then \
			echo "  CONF OK   " $(notdir $@) ; \
		else \
			echo "  CONF RE   " $(notdir $@) ; \
			cp $< $@ ; \
		fi ; \
	else \
		echo "  CONF      $@" ; \
		cp $< $@ ; \
	fi

# Here is a special case for the dependency file, this is because we
# want to always create the deps file when reloading the
# configuration, and we have a special handling in rules_main.mk for
# deps, in order to avoid missing files and circular dependencies.
# That means we can safely create the .deps directly in its final
# place, and not go through the reconf thing.

$(CONFIG_TMP_FILES): $(CONF)
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH) \
		--input=$(CONF)					\
		--python=$(CONF_TMP_BASE).py		\
		--m4=$(CONF_TMP_BASE).m4			\
		--header=$(CONF_TMP_BASE).h			\
		--depmakefile=$(CONF_DIR)/.config.deps			\
		--makefile=$(CONF_TMP_BASE).mk

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
