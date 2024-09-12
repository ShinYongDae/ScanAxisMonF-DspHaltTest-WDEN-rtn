/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig711enc.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G711 encode algorithm.
 */
#ifndef IG711ENC_
#define IG711ENC_

#include <xdas.h>
#include <ialg.h>
#include <ig711.h>

/*
 *  ======== IG711ENC_Obj ========
 *  This structure must be the first field of all G711ENC instance objects.
 */
typedef struct IG711ENC_Obj {
    struct IG711ENC_Fxns *fxns;
} IG711_Obj;

/*
 *  ======== IG711ENC_Handle ========
 *  This handle is used to reference all G711ENC instance objects.
 */
typedef struct IG711ENC_Obj *IG711ENC_Handle;

/*
 *  ======== IG711ENC_Params ========
 *  This structure defines the creation parameters for all G711ENC instance
 *  objects.
 */
typedef struct IG711ENC_Params {
    Int          size;          /* Size of this structure */ 
    XDAS_UInt16  frameLen;      /* Number of samples in a input frame */
    IG711_Mode   mode;          /* Format of the encoded output samples */
} IG711ENC_Params;

extern const IG711ENC_Params IG711ENC_PARAMS;       /* default params */

/*
 *  ======== IG711ENC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG711ENC_Status {
    Int          size;          /* Size of this structure */ 
    XDAS_UInt16  frameLen;      /* Num samples in the in frame (Read/Write) */  
    IG711_Mode   mode;          /* Format of encoded out samples (Read-Only) */
} IG711ENC_Status;

/*
 *  ======== IG711ENC_Fxns ========
 *  This structure defines all of the operations on the G711ENC instance object.
 */
typedef struct IG711ENC_Fxns {
    IALG_Fxns   ialg;    /* IG711ENC extends IALG */
    XDAS_Bool (*control)(IG711ENC_Handle handle, IG711_Cmd cmd, IG711ENC_Status *status);
    XDAS_Int16  (*encode)(IG711ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);
} IG711ENC_Fxns;

#endif  /* IG711ENC_ */
