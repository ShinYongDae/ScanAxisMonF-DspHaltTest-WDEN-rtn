/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir_ti_ialg.c ========
 *  FIR Filter Module - TI implementation of a FIR filter algorithm
 *
 *  This file contains an implementation of the IALG interface
 *  required by XDAS.
 */
#pragma CODE_SECTION(FIR_TI_activate, ".text:algActivate")
#pragma CODE_SECTION(FIR_TI_alloc, ".text:algAlloc")
#pragma CODE_SECTION(FIR_TI_deactivate, ".text:algDeactivate")
#pragma CODE_SECTION(FIR_TI_free, ".text:algFree")
#pragma CODE_SECTION(FIR_TI_initObj, ".text:algInit")
#pragma CODE_SECTION(FIR_TI_moved, ".text:algMoved")

#include <std.h>

#include <ialg.h>
#include <ifir.h>
#include <fir_ti.h>
#include <fir_ti_priv.h>

#include <string.h>         /* memcpy() declaration */

#define HISTORY 1
#define WORKBUF 2
#define NUMBUFS 3

/*
 *  ======== FIR_TI_activate ========
 *  Copy filter history from external slow memory into working buffer.
 */
Void FIR_TI_activate(IALG_Handle handle)
{
    FIR_TI_Obj *fir = (Void *)handle;

    /* copy saved history to working buffer */
    memcpy((Void *)fir->workBuf, (Void *)fir->history,
        fir->filterLenM1 * sizeof(Int));
}

/*
 *  ======== FIR_TI_alloc ========
 */
Int FIR_TI_alloc(const IALG_Params *algParams,
                 IALG_Fxns **pf, IALG_MemRec memTab[])
{
    const IFIR_Params *params = (Void *)algParams;

    if (params == NULL) {
        params = &IFIR_PARAMS;  /* set default parameters */
    }

    /* Request memory for FIR object */
    memTab[0].size = sizeof(FIR_TI_Obj);
    memTab[0].alignment = 0;
    memTab[0].space = IALG_EXTERNAL;
    memTab[0].attrs = IALG_PERSIST;

    /*
     *  Request memory filter's "inter-frame" state (i.e., the
     *  delay history)
     *
     *  Note we could have simply added the delay buffer size to the
     *  end of the FIR object by combining this request with the one
     *  above, thereby saving some code.  We separate it here for
     *  clarity.
     */
    memTab[HISTORY].size = (params->filterLen - 1) * sizeof(Int);
    memTab[HISTORY].alignment = 0;
    memTab[HISTORY].space = IALG_EXTERNAL;
    memTab[HISTORY].attrs = IALG_PERSIST;

    /*
     *  Request memory for shared working buffer
     */
    memTab[WORKBUF].size =
        (params->filterLen - 1 + params->frameLen) * sizeof(Int);
    memTab[WORKBUF].alignment = 0;
    memTab[WORKBUF].space = IALG_DARAM0;
    memTab[WORKBUF].attrs = IALG_SCRATCH;
    
    return (NUMBUFS);
}

/*
 *  ======== FIR_TI_deactivate ========
 *  Copy filter history from working buffer to external memory
 */
Void FIR_TI_deactivate(IALG_Handle handle)
{
    FIR_TI_Obj *fir = (Void *)handle;

    /* copy history to external history buffer */
    memcpy((Void *)fir->history, (Void *)fir->workBuf,
        fir->filterLenM1 * sizeof(Int));
}

/*
 *  ======== FIR_TI_free ========
 */
Int FIR_TI_free(IALG_Handle handle, IALG_MemRec memTab[])
{
    FIR_TI_Obj *fir = (Void *)handle;

    FIR_TI_alloc(NULL, NULL, memTab);

    memTab[0].base = handle;
    
    memTab[HISTORY].base = fir->history;
    memTab[HISTORY].size = fir->filterLenM1 * sizeof(Int);

    memTab[WORKBUF].size =
        (fir->filterLenM1 + fir->frameLen) * sizeof(Int);
    memTab[WORKBUF].base = fir->workBuf;

    return (NUMBUFS);
}

/*
 *  ======== FIR_TI_initObj ========
 */
Int FIR_TI_initObj(IALG_Handle handle,
                const IALG_MemRec memTab[], IALG_Handle p,
                const IALG_Params *algParams)
{
    FIR_TI_Obj *fir = (Void *)handle;
    const IFIR_Params *params = (Void *)algParams;

    if (params == NULL) {
        params = &IFIR_PARAMS;  /* set default parameters */
    }

    fir->coeff = params->coeffPtr;
    fir->workBuf = memTab[WORKBUF].base;
    fir->history = memTab[HISTORY].base;
    fir->filterLenM1 = params->filterLen - 1;
    fir->frameLen = params->frameLen;

    return (IALG_EOK);
}

/*
 *  ======== FIR_TI_moved ========
 */
Void FIR_TI_moved(IALG_Handle handle,
                const IALG_MemRec memTab[], IALG_Handle p,
                const IALG_Params *algParams)
{
    FIR_TI_Obj *fir = (Void *)handle;
    const IFIR_Params *params = (Void *)algParams;

    if (params != NULL) {
        fir->coeff = params->coeffPtr;
    }

    fir->workBuf = memTab[WORKBUF].base;
    fir->history = memTab[HISTORY].base;
}

Void FIR_TI_init(Void)
{

}

Void FIR_TI_exit(Void)
{

}

