/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir.h ========
 *  This header defines all types, constants, and functions used by 
 *  applications that use the FIR algorithm.
 *
 *  Applications that use this interface enjoy type safety and
 *  the ability to incorporate multiple implementations of the FIR
 *  algorithm in a single application at the expense of some
 *  additional indirection.
 */
#ifndef FIR_
#define FIR_

#include <alg.h>
#include <ifir.h>
#include <ialg.h>

/*
 *  ======== FIR_Handle ========
 *  FIR algorithm instance handle
 */
typedef struct IFIR_Obj *FIR_Handle;

/*
 *  ======== FIR_Params ========
 *  FIR algorithm instance creation parameters
 */
typedef struct IFIR_Params FIR_Params;

/*
 *  ======== FIR_PARAMS ========
 *  Default instance parameters
 */
#define FIR_PARAMS IFIR_PARAMS

/*
 *  ======== FIR_apply ========
 *  Apply a FIR filter to the input array and place results in the
 *  output array.
 */
extern Void FIR_apply(FIR_Handle fir, Int in[], Int out[]);

/*
 *  ======== FIR_create ========
 *  Create an instance of a FIR object.
 */
static inline FIR_Handle FIR_create(const IFIR_Fxns *fxns, const FIR_Params *prms)
{
    return ((FIR_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== FIR_delete ========
 *  Delete a FIR instance object
 */
static inline Void FIR_delete(FIR_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}


/*
 *  ======== FIR_exit ========
 *  Module finalization
 */
extern Void FIR_exit(Void);

/*
 *  ======== FIR_init ========
 *  Module initialization
 */
extern Void FIR_init(Void);

#endif  /* FIR_ */
