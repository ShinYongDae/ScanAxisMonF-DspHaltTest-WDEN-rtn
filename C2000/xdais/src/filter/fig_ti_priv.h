/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fig_ti_priv.h ========
 *  Internal vendor specific (TI) interface header for FIG
 *  algorithm. Only the implementation source files include
 *  this header; this header is not shipped as part of the
 *  algorithm.
 *
 *  This header contains declarations that are specific to
 *  this implementation and which do not need to be exposed
 *  in order for an application to use the FIG algorithm.
 */
#ifndef FIG_TI_PRIV
#define FIG_TI_PRIV

#include <ialg.h>

typedef struct FIG_TI_Obj {
    IALG_Obj    alg;            /* MUST be first field of all XDAS algs */
    Int         *coeff;         /* on-chip persistant coeffient array */
    Int         filterLen;      /* filter length (in words) */
} FIG_TI_Obj;


extern Int FIG_TI_alloc(const IALG_Params *, IALG_Fxns **, IALG_MemRec *);

extern Int FIG_TI_free(IALG_Handle, IALG_MemRec *);

extern Void FIG_TI_getStatus(IFIG_Handle handle, IFIG_Status *status);

extern Int FIG_TI_initObj(IALG_Handle,
                    const IALG_MemRec *, IALG_Handle, const IALG_Params *);

extern Void FIG_TI_moved(IALG_Handle,
                    const IALG_MemRec *, IALG_Handle, const IALG_Params *);

#endif
