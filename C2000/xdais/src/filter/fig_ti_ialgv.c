/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fig_ti_ialgvt.c ========
 *  This file contains the function table definitions for all interfaces
 *  implemented by the FIG_TI module.
 */
#include <std.h>
#include <ialg.h>
#include <ifig.h>
#include <fig_ti.h>
#include <fig_ti_priv.h>

#define IALGFXNS \
    &FIG_TI_IALG,   /* implementation ID */                         \
    NULL,           /* activate (NULL => nothing to do) */          \
    FIG_TI_alloc,   /* alloc */                                     \
    NULL,           /* control (NULL => no control operations) */   \
    NULL,           /* deactivate (NULL => nothing to do) */        \
    FIG_TI_free,    /* free */                                      \
    FIG_TI_initObj, /* init */                                      \
    FIG_TI_moved,   /* moved */                                     \
    NULL            /* numAlloc (NULL => IALG_MAXMEMRECS) */        \
    
/*
 *  ======== FIG_TI_IFIG ========
 */
IFIG_Fxns FIG_TI_IFIG = {       /* module_vendor_interface */
    IALGFXNS,               /* IALG functions */
    FIG_TI_getStatus        /* IFIG getStatus */
};

/*
 *  ======== FIG_TI_IALG ========
 *  This structure defines TI's implementation of the IALG interface
 *  for the FIG_TI module.
 */
#ifdef _TI_

asm("_FIG_TI_IALG .set _FIG_TI_IFIG");

#else

/*
 *  We duplicate the structure here to allow this code to be compiled and
 *  run non-DSP platforms at the expense of unnecessary data space
 *  consumed by the definition below.
 */
IALG_Fxns FIG_TI_IALG = {       /* module_vendor_interface */
    IALGFXNS,               /* IALG functions */
};

#endif
