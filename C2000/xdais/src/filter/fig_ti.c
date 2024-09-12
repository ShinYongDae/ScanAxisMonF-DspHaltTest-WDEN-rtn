/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fig_ti.c ========
 *  Filter Group - this module implements a filter group; a group of FIR
 *  filters that share a common set of coefficients and a working buffer.
 */
#pragma CODE_SECTION(FIG_TI_alloc, ".text:algAlloc")
#pragma CODE_SECTION(FIG_TI_free, ".text:algFree")
#pragma CODE_SECTION(FIG_TI_initObj, ".text:algInit")
#pragma CODE_SECTION(FIG_TI_moved, ".text:algMoved")

#include <std.h>
#include <ialg.h>
#include <fig_ti.h>
#include <ifig.h>
#include <string.h>     /* memcpy() declaration */

#define COEFF   1
#define NUMBUFS 2

typedef struct FIG_TI_Obj {
    IALG_Obj    alg;            /* MUST be first field of all XDAS algs */
    Int         *coeff;         /* on-chip persistant coeffient array */
    Int         filterLen;      /* filter length (in words) */
} FIG_TI_Obj;

/*
 *  ======== FIG_TI_alloc ========
 */
Int FIG_TI_alloc(const IALG_Params *algParams, IALG_Fxns **parentFxns,
                 IALG_MemRec memTab[])
{
    const IFIG_Params *params = (Void *)algParams;

    if (params == NULL) {
        params = &IFIG_PARAMS;  /* set default parameters */
    }

    /* Request memory for FIG object */
    memTab[0].size = sizeof (FIG_TI_Obj);
    memTab[0].alignment = 0;
    memTab[0].space = IALG_EXTERNAL;
    memTab[0].attrs = IALG_PERSIST;

    /*
     *  Request memory for filter coefficients
     *
     *  Note that this buffer is declared as persistent; i.e., it is the
     *  responsibility of the client to insure that its contents are
     *  preserved whenever this object is active.
     */
    memTab[COEFF].size = params->filterLen * sizeof(Int);
    memTab[COEFF].alignment = 0;
    memTab[COEFF].space = IALG_DARAM1;
    memTab[COEFF].attrs = IALG_PERSIST;

    return (NUMBUFS);
}

/*
 *  ======== FIG_TI_free ========
 */
Int FIG_TI_free(IALG_Handle handle, IALG_MemRec memTab[])
{
    FIG_TI_Obj *fig = (Void *)handle;

    FIG_TI_alloc(NULL, NULL, memTab);

    memTab[0].base = handle;
    
    memTab[COEFF].base = fig->coeff;
    memTab[COEFF].size = fig->filterLen * sizeof (Int);
    
    return (NUMBUFS);
}

/*
 *  ======== FIG_TI_initObj ========
 */
Int FIG_TI_initObj(IALG_Handle handle,
                    const IALG_MemRec memTab[], IALG_Handle parent,
                    const IALG_Params *algParams)
{
    FIG_TI_Obj *fig = (Void *)handle;
    const IFIG_Params *params = (Void *)algParams;

    if (params == NULL) {
        params = &IFIG_PARAMS;  /* use defaults if algParams == NULL */
    }
    
    /* initialize the FIG object's fields */
    fig->coeff = memTab[COEFF].base;
    fig->filterLen = params->filterLen;

    /* copy coefficients into on-chip persistant memory */
    memcpy((Void *)fig->coeff,
        (Void *)params->coeffPtr, params->filterLen * sizeof (Int));

    return (IALG_EOK);
}

/*
 *  ======== FIG_TI_getStatus ========
 */
Void FIG_TI_getStatus(IFIG_Handle handle, IFIG_Status *status)
{
    FIG_TI_Obj *fig = (Void *)handle;

    status->coeffPtr = fig->coeff;
}

/*
 *  ======== FIG_TI_moved ========
 */
Void FIG_TI_moved(IALG_Handle handle,
                    const IALG_MemRec memTab[], IALG_Handle parent,
                    const IALG_Params *algParams)
{
    FIG_TI_Obj *fig = (Void *)handle;
    
    /* initialize the FIG object's fields */
    fig->coeff = memTab[COEFF].base;
}

/*
 *  ======== FIG_TI_exit ========
 */
 
Void FIG_TI_exit(Void)
{

}


/*
 *  ======== FIG_TI_init ========
 */
Void FIG_TI_init(Void)
{

}

