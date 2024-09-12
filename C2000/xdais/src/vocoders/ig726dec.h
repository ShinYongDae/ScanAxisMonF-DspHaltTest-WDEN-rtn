/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig726dec.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G726 decode algorithm.
 */
#ifndef IG726DEC_
#define IG726DEC_

#include <xdas.h>
#include <ialg.h>
#include <ig726.h>

/*
 *  ======== IG726DEC_Obj ========
 *  This structure must be the first field of all G726DEC instance objects.
 */
typedef struct IG726DEC_Obj {
    struct IG726DEC_Fxns *fxns;
} IG726DEC_Obj;

/*
 *  ======== IG726DEC_Handle ========
 *  This handle is used to reference all G726DEC instance objects.
 */
typedef struct IG726DEC_Obj *IG726DEC_Handle;

/*
 *  ======== IG726DEC_Params ========
 *  This structure defines the creation parameters for all G726DEC objects
 */
typedef struct IG726DEC_Params {
    Int         size;        /* Size of this structure */ 
    XDAS_UInt16 frameLen;    /* Length of output buffer */
    IG726_Mode  mode;        /* Format of the encoded buffer */
    IG726_Rate  rate;        /* Working rate */
} IG726DEC_Params;

extern const IG726DEC_Params IG726DEC_PARAMS;       /* default params */

/*
 *  ======== IG726DEC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG726DEC_Status {
    Int         size;        /* Size of this structure */ 
    XDAS_UInt16 frameLen;    /* Length of input buffer (Read/Write) */
    IG726_Mode  mode;        /* Format of the decoded buffer (Read-Only) */
    IG726_Rate  rate;        /* Working rate (Read-Only) */
} IG726DEC_Status;

/*
 *  ======== IG726DEC_Fxns ========
 *  This structure defines all of the operations on G726DEC objects.
 */
typedef struct IG726DEC_Fxns {
    IALG_Fxns   ialg;    /* IG726DEC extends IALG */
    XDAS_Bool  (*control)(IG726DEC_Handle handle, IG726_Cmd cmd , IG726DEC_Status *status);
    XDAS_Int16  (*decode)(IG726DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);
} IG726DEC_Fxns;

#endif  /* IG726DEC_ */
