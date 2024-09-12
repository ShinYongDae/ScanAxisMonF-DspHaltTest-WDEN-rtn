/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig728enc.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G728ENC encode algorithm.
 */
#ifndef IG728ENC_
#define IG728ENC_

#include <xdas.h>
#include <ialg.h>
#include <ig728.h>

/*
 *  ======== IG728ENC_Handle ========
 *  This handle is used to reference all G728ENC instance objects
 */
typedef struct IG728ENC_Obj *IG728ENC_Handle;

/*
 *  ======== IG728ENC_Obj ========
 *  This structure must be the first field of all G728ENC instance objects
 */
typedef struct IG728ENC_Obj {
    struct IG728ENC_Fxns *fxns;
} IG728ENC_Obj;

/*
 *  ======== IG728ENC_Params ========
 *  This structure defines the creation parameters for all G728ENC objects
 */
typedef struct IG728ENC_Params {
    Int         size;        /* Size of this structure */ 
    IG728_Mode  mode;        /* Format of the in buffer */
    XDAS_Bool   pwfEnable;   /* Enable/Disable perceptual weighting filter */
    XDAS_UInt8  syncPeriod;  /* Positive value measured in codewords, */
} IG728ENC_Params;           /* zero disables inband sync */

extern const IG728ENC_Params IG728ENC_PARAMS;       /* default params */

/*
 *  ======== IG728ENC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG728ENC_Status {
    Int         size;        /* Size of this structure */ 
    IG728_Mode  mode;        /* Format of the in buffer (Read) */
    XDAS_Bool   pwfEnable;   /* Enable/Disable pw filter (Read/Write) */
    XDAS_UInt8  syncPeriod;  /* Positive value measured in codewords, */
} IG728ENC_Status;           /* zero disables inband sync (Read/Write) */ 

/*
 *  ======== IG728ENC_Fxns ========
 *  This structure defines all of the operations on G728ENC objects
 */
typedef struct IG728ENC_Fxns {
    IALG_Fxns  ialg;    /* IG728ENC extends IALG */
    XDAS_Bool  (*control) (IG728ENC_Handle handle, IG728_Cmd cmd, IG728ENC_Status *status);
    XDAS_Int8  (*encode)(IG728ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);
} IG728ENC_Fxns;

#endif  /* IG728ENC_ */
