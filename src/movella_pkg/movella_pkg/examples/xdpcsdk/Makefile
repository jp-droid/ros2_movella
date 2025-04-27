MOVELLA_INSTALL_PREFIX?=$(shell dirname $$(dirname $$(pwd)))

COMMON_TARGETS=cpp
TARGETS=$(COMMON_TARGETS)
CLEAN_COMMON_TARGETS=$(patsubst %,%.clean,$(COMMON_TARGETS))
CLEAN_TARGETS=$(patsubst %,%.clean,$(TARGETS))

.PHONY: all $(TARGETS) clean
all: $(TARGETS)

$(TARGETS):
	$(MAKE) -C $@

$(CLEAN_COMMON_TARGETS):
	-$(MAKE) -C $(patsubst %.clean,%,$@) clean

clean: $(CLEAN_TARGETS)
