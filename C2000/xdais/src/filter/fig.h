/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fig.h ========
 *  Filter Group Module Header - This module implements a FIR filter
 *  group object.  A filter group object simply maintains global state
 *  (common coefficients and working buffer) multiple FIR objects.
 *  Thus, this module does not have a "process" method, it only
 *  implements "activate" and "deactivate".
 */
#ifndef FIG_
#define FIG_

#include <alg.h>
#include <ialg.h>
#include <ifig.h>

typedef struct IFIG_Obj *FIG_Handle;

/*
 *  ======== FIG_Params ========
 *  Filter group instance creation parameters
 */
typedef struct IFIG_Params FIG_Params;

/*
 *  ======== FIG_PARAMS ========
 *  Default FIG instance parameters
 */
#define FIG_PARAMS IFIG_PARAMS

/*
 *  ======== FIG_Status ========
 *  Status structure for getting FIG instance attributes
 */
typedef struct IFIG_Status FIG_Status;

/*
 *  ======== FIG_activate ========
 */
static inline Void FIG_activate(FIG_Handle handle)
{
    ALG_activate((ALG_Handle)handle);
}

/*
 *  ======== FIG_create ========
 */
static inline FIG_Handle FIG_create(IFIG_Fxns *fxns, IFIG_Params *prms)
{
    return ((FIG_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== FIG_deactivate ========
 */
static inline Void FIG_deactivate(FIG_Handle handle)
{
    ALG_deactivate((ALG_Handle)handle);
}

/*
 *  ======== FIG_delete ========
 */
static inline Void FIG_delete(FIG_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== FIG_exit ========
 */
extern Void FIG_exit(Void);

/*
 *  ======== FIG_getStatus ========
 */
static inline Void FIG_getStatus(FIG_Handle handle, FIG_Status *status)
{
    handle->fxns->getStatus(handle, status);
}

/*
 *  ======== FIG_init ========
 */
Void FIG_init(Void);

#endif  /* FIG_ */
