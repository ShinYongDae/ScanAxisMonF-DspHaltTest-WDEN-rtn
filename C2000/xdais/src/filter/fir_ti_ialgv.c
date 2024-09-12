/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir_ti_ialgvt.c ========
 *  This file contains the function table definitions for all
 *  interfaces implemented by the FIR_TI module that derive
 *  from IALG
 *
 *  We place these tables in a separate file for two reasons:
 *     1. We want allow one to one to replace these tables
 *        with different definitions.  For example, one may
 *        want to build a system where the FIR is activated
 *        once and never deactivated, moved, or freed.
 *
 *     2. Eventually there will be a separate "system build"
 *        tool that builds these tables automatically 
 *        and if it determines that only one implementation
 *        of an API exists, "short circuits" the vtable by
 *        linking calls directly to the algorithm's functions.
 */
#include <std.h>

#include <ialg.h>
#include <ifir.h>

#include <fir_ti.h>
#include <fir_ti_priv.h>

#define IALGFXNS \
    &FIR_TI_IALG,       /* module ID */                         \
    FIR_TI_activate,    /* activate */                          \
    FIR_TI_alloc,       /* alloc */                             \
    NULL,               /* control (NULL => no control ops) */  \
    FIR_TI_deactivate,  /* deactivate */                        \
    FIR_TI_free,        /* free */                              \
    FIR_TI_initObj,     /* init */                              \
    FIR_TI_moved,       /* moved */                             \
    NULL                /* numAlloc (NULL => IALG_MAXMEMRECS) */\

/*
 *  ======== FIR_TI_IFIR ========
 *  This structure defines TI's implementation of the IFIR interface
 *  for the FIR_TI module.
 */
IFIR_Fxns FIR_TI_IFIR = {       /* module_vendor_interface */
    IALGFXNS,
    FIR_TI_filter    /* filter */
};

/*
 *  ======== FIR_TI_IALG ========
 *  This structure defines TI's implementation of the IALG interface
 *  for the FIR_TI module.
 */
#ifdef _TI_

asm("_FIR_TI_IALG .set _FIR_TI_IFIR");

#else

/*
 *  We duplicate the structure here to allow this code to be compiled and
 *  run non-DSP platforms at the expense of unnecessary data space
 *  consumed by the definition below.
 */
IALG_Fxns FIR_TI_IALG = {       /* module_vendor_interface */
    IALGFXNS
};

#endif

