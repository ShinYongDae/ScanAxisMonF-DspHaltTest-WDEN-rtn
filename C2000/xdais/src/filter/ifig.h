/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ifig.h ========
 *  Filter Group Module Header - This module implements a FIR filter
 *  group object.  A filter group object simply maintains global state
 *  (common coefficients and working buffer) multiple FIR objects.
 *  Thus, this module does not have a "process" method, it only
 *  implements "activate" and "deactivate".
 */
#ifndef IFIG_
#define IFIG_

#include <ialg.h>

/*
 *  ======== IFIG_Params ========
 *  Filter group instance creation parameters
 */
typedef struct IFIG_Params {
    Int size;           /* sizeof this structure */
    Int *coeffPtr;      /* pointer to coefficient array */
    Int filterLen;      /* length of coefficient array (words) */
} IFIG_Params;

extern const IFIG_Params IFIG_PARAMS;   /* default instance parameters */

/*
 *  ======== IFIG_Obj ========
 */
typedef struct IFIG_Obj {
    struct IFIG_Fxns *fxns;
} IFIG_Obj;

/*
 *  ======== IFIG_Handle ========
 */
typedef struct IFIG_Obj *IFIG_Handle;

/*
 *  ======== IFIG_Status ========
 *  Status structure for getting FIG instance attributes
 */
typedef struct IFIG_Status {
    Int *coeffPtr;          /* pointer to coefficient array */
} IFIG_Status;

/*
 *  ======== IFIG_Fxns ========
 */
typedef struct IFIG_Fxns {
    IALG_Fxns ialg;
    Void (*getStatus)(IFIG_Handle handle, IFIG_Status *status);
} IFIG_Fxns;

#endif  /* IFIG_ */
