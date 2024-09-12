#
#  ======== makefile ========
#
#  This "top level" make file can be used with GNU's make utility
#  to (re)build XDAS examples
#
#  To (re)build from scratch type the following:
#	gmake clean
#	gmake 
#
#
#  subdirs = 	list of all sub-directories containing a makefile
#  submake = 	a definition to perform a sub-make with an embedded new
#		line.  Do not remove the blank line!  This line is required
#		to ensure that the $(foreach ...) function used below 
#		generates a separate line for each sub-directory
#  makeall = 	recursive sub-make for each directory defined in $(subdirs)
#
include chip.gmk

subdirs = $(dir $(wildcard */makefile))

define submake28l
$(MAKE) CHIP=28l -C $(var) $@

endef

define submake54
$(MAKE) CHIP=54 -C $(var) $@

endef

define submake54f
$(MAKE) CHIP=54f -C $(var) $@

endef

define submake55
$(MAKE) CHIP=55 -C $(var) $@

endef

define submake55l
$(MAKE) CHIP=55l -C $(var) $@

endef

define submake62
$(MAKE) CHIP=62 -C $(var) $@

endef

define submake64
$(MAKE) CHIP=64 -C $(var) $@

endef

define submake67
$(MAKE) CHIP=67 -C $(var) $@

endef

makeall = $(foreach var,$(subdirs),$(submake))

makeall28l = $(foreach var,$(subdirs),$(submake28l))

makeall54 = $(foreach var,$(subdirs),$(submake54))

makeall54f = $(foreach var,$(subdirs),$(submake54f))

makeall55 = $(foreach var,$(subdirs),$(submake55))

makeall55l = $(foreach var,$(subdirs),$(submake55l))

makeall62 = $(foreach var,$(subdirs),$(submake62))

makeall64 = $(foreach var,$(subdirs),$(submake64))

makeall67 = $(foreach var,$(subdirs),$(submake67))


all:
	@echo making all files in the directories $(subdirs) ...
ifneq (,$(findstring 28,$(CHIP)))
	@echo making all 28 files ...
	+@$(makeall28l)
endif
ifneq (,$(findstring 54,$(CHIP)))
	@echo making all 54 files ...
	+@$(makeall54)
	@echo making all 54f files ...
	+@$(makeall54f)
endif
ifneq (,$(findstring 55,$(CHIP)))
	@echo making all 55l files ...
	+@$(makeall55l)
endif
ifneq (,$(findstring 6,$(CHIP)))
	@echo making all 62 files ...
	+@$(makeall62)
	@echo making all 64 files ...
	+@$(makeall64)
	@echo making all 67 files ...
	+@$(makeall67)
endif


clean::
	@echo "cleaning $(subdirs) ..."
ifneq (,$(findstring 28,$(CHIP)))
	@echo cleaning all 28 files ...
	+@$(makeall28l)
endif
ifneq (,$(findstring 54,$(CHIP)))
	@echo cleaning all 54 files ...
	+@$(makeall54)
	@echo cleaning all 54f files ...
	+@$(makeall54f)
endif
ifneq (,$(findstring 55,$(CHIP)))
	@echo cleaning all 55 files ...
	+@$(makeall55)
	@echo cleaning all 55l files ...
	+@$(makeall55l)
endif
ifneq (,$(findstring 6,$(CHIP)))
	@echo cleaning all 62 files ...
	+@$(makeall62)
	@echo cleaning all 64 files ...
	+@$(makeall64)
	@echo cleaning all 67 files ...
	+@$(makeall67)
endif
