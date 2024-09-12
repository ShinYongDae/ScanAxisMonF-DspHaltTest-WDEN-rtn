#
#  ======== makefile ========
#  Makefile for the API modules ALG and TST
#

#
#  Set DEBUG to 1 so it is easier to follow the examples with the 
#  debugger.  Note this must be done prior to including the xdais.gmk 
#  file; xdais.gmk references this variable.
#
DEBUG = 1

include chip.gmk
include $(TI_DIR)/$(CHIPDIR)/xdais/include/xdais.gmk

OBJS    = alg_control.o$(CHIP) alg_create.o$(CHIP) alg_malloc.o$(CHIP)

OLIBS	= $L/api.l$(CHIP) 

#
#  Targets ...
#

all: $(OLIBS)

clean::
	@remove $(OBJS) $(OLIBS)

$L/api.l$(CHIP): $(OBJS)
	$(ARCH) $@ $(OBJS)

#
#  Dependencies ...
#

