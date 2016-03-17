SRC:=source
BUILD:=build
SOURCES:=$(shell find $(SRC) -name \*.rst)
build-outputs=$(shell find $(BUILD)/$(1) -name \*.$(2) 2> /dev/null) __build_$(1)
BUILDERS=html latex

all: $(BUILDERS)

clean:
	rm -r $(foreach b,$(BUILDERS),$(BUILD)/$(b))

define latex-post-build
	make -C $(BUILD)/latex all-pdf
endef

define build-def

$(1): $$(call build-outputs,$(1),$(2))

$$(call build-outputs,$(1),$(2)): _build_$(1)

$(BUILD)/$(1):
	test -d $$@ || mkdir -p $$@

_build_$(1): $(SOURCES) $(BUILD)/$(1)
	sphinx-build -b $(1) -n $(SRC) $(BUILD)/$(1)
	$$(call $(1)-post-build)

endef


$(eval $(foreach b,$(BUILDERS),$(call build-def,$(b),$(b))))
