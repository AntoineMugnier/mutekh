
FORCE:
	@true

.PHONY: FORCE

config:
	@true

checkconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --check		\
		--build=$(BUILD)

findbadtokens:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --find-bad-tokens		\
		--build=$(BUILD)

listconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --list		\
		--build=$(BUILD)


listflatconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --list=flat		\
		--build=$(BUILD)


listallconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --list=all		\
		--build=$(BUILD)

listinit:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --list=init		\
		--build=$(BUILD)

showconfig:
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--src-path=$(CONF_PATH):$(MUTEK_SRC_DIR) \
		--input=$(CONF) --info=$(TOKEN)		\
		--build=$(BUILD)


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
