/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig726enc.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G726 encode algorithm.
 */
#ifndef IG726ENC_
#define IG726ENC_

#include <xdas.h>
#include <ialg.h>
#include <ig726.h>

/*
 *  ======== IG726ENC_Obj ========
 *  This structure must be the first field of all G726ENC instance objects.
 */
typedef struct IG726ENC_Obj {
    struct IG726ENC_Fxns *fxns;
} IG726ENC_Obj;

/*
 *  ======== IG726ENC_Handle ========
 *  This handle is used to reference all G726ENC instance objects.
 */
typedef struct IG726ENC_Obj *IG726ENC_Handle;

/*
 *  ======== IG726ENC_Params ========
 *  This structure defines the creation parameters for all G726ENC objects
 */
typedef struct IG726ENC_Params {
    Int         size;        /* Size of this structure */ 
    XDAS_UInt16 frameLen;    /* Length of output buffer */
    IG726_Mode  mode;        /* Format of the encoded buffer */
    IG726_Rate  rate;        /* Working rate */
} IG726ENC_Params;

extern const IG726ENC_Params IG726ENC_PARAMS;       /* default params */

/*
 *  ======== IG726ENC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG726ENC_Status {
    Int         size;        /* Size of this structure */ 
    XDAS_UInt16 frameLen;    /* Length of input buffer (Read/Write) */
    IG726_Mode  mode;        /* Format of the decoded buffer (Read-Only) */
    IG726_Rate  rate;        /* Working rate (Read-Only) */
} IG726ENC_Status;

/*
 *  ======== IG726ENC_Fxns ========
 *  This structure defines all of the operations on G726ENC objects.
 */
typedef struct IG726ENC_Fxns {
    IALG_Fxns   ialg;    /* IG726ENC extends IALG */
    XDAS_Bool  (*control)(IG726ENC_Handle handle, IG726_Cmd cmd , IG726ENC_Status *status);
    XDAS_Int16  (*encode)(IG726ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);
} IG726ENC_Fxns;

#endif  /* IG726ENC_ */
