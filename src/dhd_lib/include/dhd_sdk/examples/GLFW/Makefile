#  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  Force Dimension SDK 3.11.1


# DO NOT MODIFY THIS FILE
# project-specific changes should go in Makefile.project(.*)


TOP_DIR = ../..
include $(TOP_DIR)/Makefile.common

SUBDIRS = $(patsubst %/,%,$(sort $(dir $(wildcard */Makefile))))

ifneq (,$(wildcard ./Makefile.project))
include ./Makefile.project
endif

all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
