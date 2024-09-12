#
#  ======== makefile ========
#  Makefile for the Line Echo Canceler API library
#

#
#  Set DEBUG to 1 so it is easier to follow the examples with the 
#  debugger.  Note this must be done prior to including the xdais.gmk 
#  file; xdais.gmk references this variable.
#
DEBUG = 1

include chip.gmk
include $(TI_DIR)/$(CHIPDIR)/xdais/include/xdais.gmk


SPIOBJS = ilec.o$(CHIP)

APIOBJS = lec.o$(CHIP)

OLIBS	= $L/api.l$(CHIP)

#
#  Targets ...
#

all: $(OLIBS) 

$L/api.l$(CHIP): $(APIOBJS) $(SPIOBJS)
	$(ARCH) $@ $(APIOBJS) $(SPIOBJS)

clean::
	@remove $(APIOBJS) $(SPIOBJS)

#
#  Dependencies ...
#

