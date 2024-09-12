/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir_ti_priv.h ========
 *  Internal vendor specific (TI) interface header for FIR
 *  algorithm. Only the implementation source files include
 *  this header; this header is not shipped as part of the
 *  algorithm.
 *
 *  This header contains declarations that are specific to
 *  this implementation and which do not need to be exposed
 *  in order for an application to use the FIR algorithm.
 */
#ifndef FIR_TI_PRIV_
#define FIR_TI_PRIV_

#include <ialg.h>
#include <ifir.h>

typedef struct FIR_TI_Obj {
    IALG_Obj    alg;            /* MUST be first field of all XDAS algs */
    Int         *workBuf;       /* on-chip scratch history */
    Int         *coeff;         /* on-chip persistant coeff */
    Int         *history;       /* off chip presistant history */
    Int         filterLenM1;    /* length of coefficient array - 1 */
    Int         frameLen;       /* length of input (output) buffer */
} FIR_TI_Obj;

extern Void FIR_TI_activate(IALG_Handle handle);

extern Void FIR_TI_deactivate(IALG_Handle handle);

extern Int FIR_TI_alloc(const IALG_Params *algParams, IALG_Fxns **pf,
                        IALG_MemRec memTab[]);

extern Int FIR_TI_free(IALG_Handle handle, IALG_MemRec memTab[]);

extern Int FIR_TI_initObj(IALG_Handle handle,
                          const IALG_MemRec memTab[], IALG_Handle parent,
                          const IALG_Params *algParams);
                
extern Void FIR_TI_moved(IALG_Handle handle,
                          const IALG_MemRec memTab[], IALG_Handle parent,
                          const IALG_Params *algParams);
                
extern Void FIR_TI_filter(IFIR_Handle handle, Int in[], Int out[]);

#endif  /* FIR_TI_PRIV_ */
