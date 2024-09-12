/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig728dec.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G728 decode algorithm.
 */
#ifndef IG728DEC_
#define IG728DEC_

#include <xdas.h>
#include <ialg.h>
#include <ig728.h>

/*
 *  ======== IG728DEC_Handle ========
 *  This handle is used to reference all G728DEC instance objects
 */
typedef struct IG728DEC_Obj *IG728DEC_Handle;

/*
 *  ======== IG728DEC_Obj ========
 *  This structure must be the first field of all G728DEC instance objects
 */
typedef struct IG728DEC_Obj {
    struct IG728DEC_Fxns *fxns;
} IG728DEC_Obj;

/*
 *  ======== IG728DEC_Params ========
 *  This structure defines the creation parameters for all G728DEC objects
 */
typedef struct IG728DEC_Params {
    Int         size;        /* Size of this structure */ 
    IG728_Mode  mode;        /* Format of the out buffer */
    XDAS_Bool   pfoEnable;   /* Enable/Disable postfilter */
    XDAS_Int8   syncPeriod;  /* Positive value measured in codewords, */
} IG728DEC_Params;           /* zero disables inband sync */ 

extern const IG728DEC_Params IG728DEC_PARAMS;       /* default params */

/*
 *  ======== IG728DEC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG728DEC_Status {
    Int         size;        /* Size of this structure */ 
    IG728_Mode  mode;        /* Format of the out buffer (Read/Write) */
    XDAS_Bool   pfoEnable;   /* Enable/Disable postfilter (Read/Write) */
    XDAS_Int8   syncPeriod;  /* Positive value measured in codewords, */
} IG728DEC_Status;           /* zero disables inband sync (Read/Write) */

/*
 *  ======== IG728DEC_Fxns ========
 *  This structure defines all of the operations on G728DEC objects
 */
typedef struct IG728DEC_Fxns {
    IALG_Fxns  ialg;    /* IG728DEC extends IALG */
    XDAS_Bool  (*control)(IG728DEC_Handle handle, IG728_Cmd cmd, IG728DEC_Status *status);
    XDAS_Int8 (*decode)(IG728DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);
} IG728DEC_Fxns;

#endif  /* IG728DEC_ */

