#
#  ======== makefile ========
#  Makefile for building the FIR and FIG algorithms
#

#
#  Set DEBUG to 1 so it is easier to follow the examples with the 
#  debugger.  Note this must be done prior to including the xdais.gmkk 
#  file; xdais.gmk references this variable.
#
DEBUG = 1

include chip.gmk
include $(TI_DIR)/$(CHIPDIR)/xdais/include/xdais.gmk


APIOBJS = fir.o$(CHIP) fig.o$(CHIP)
SPIOBJS = ifir.o$(CHIP) ifig.o$(CHIP)

FIROBJS = ifir.o$(CHIP) fir_ti_ialg.o$(CHIP) fir_ti_ifir.o$(CHIP) \
          fir_ti_ialgv.o$(CHIP) 	  

FIGOBJS = ifig.o$(CHIP) fig_ti.o$(CHIP) fig_ti_ialgv.o$(CHIP) 

ALGLIBS = $L/fig_ti.l$(CHIP) $L/fir_ti.l$(CHIP) 
OLIBS	= $(ALGLIBS) $L/api.l$(CHIP)

#
#  Targets ...
#

all: $(OLIBS) fir_ti_ext.o$(CHIP)

$L/fir_ti.l$(CHIP): $(FIROBJS)
	$(ARCH) $@ $(FIROBJS)

$L/fig_ti.l$(CHIP): $(FIGOBJS)
	$(ARCH) $@ $(FIGOBJS)

$L/api.l$(CHIP): $(APIOBJS) $(SPIOBJS)
	$(ARCH) $@ $(APIOBJS) $(SPIOBJS)

clean::
	@$(RM) $(APIOBJS) $(SPIOBJS) $(FIROBJS) $(FIGOBJS)
	@$(RM) $(EXES) $(ALGLIBS)
