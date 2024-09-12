#
#  ======== makefile ========
#  Makefile for the Vocoder API library
#

#
#  Set DEBUG to 1 so it is easier to follow the examples with the 
#  debugger.  Note this must be done prior to including the xdais.gmk 
#  file; xdais.gmk references this variable.
#
DEBUG = 1

include chip.gmk
include $(TI_DIR)/$(CHIPDIR)/xdais/include/xdais.gmk

SPIOBJS = ig711dec.o$(CHIP) ig711enc.o$(CHIP) ig723dec.o$(CHIP) \
	  ig723enc.o$(CHIP) ig726dec.o$(CHIP) ig726enc.o$(CHIP) \
	  ig728dec.o$(CHIP) ig728enc.o$(CHIP) ig729dec.o$(CHIP) \
	  ig729enc.o$(CHIP) 

APIOBJS = g711dec.o$(CHIP) g711enc.o$(CHIP) g723dec.o$(CHIP) \
	  g723enc.o$(CHIP) g726dec.o$(CHIP) g726enc.o$(CHIP) \
	  g728dec.o$(CHIP) g728enc.o$(CHIP) g729dec.o$(CHIP) \
	  g729enc.o$(CHIP) 

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

