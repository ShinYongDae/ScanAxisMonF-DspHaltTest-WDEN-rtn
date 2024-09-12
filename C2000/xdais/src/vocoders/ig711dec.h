/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig711dec.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G711 decode algorithm.
 */
#ifndef IG711DEC_
#define IG711DEC_

#include <xdas.h>
#include <ialg.h>
#include <ig711.h>

/*
 *  ======== IG711DEC_Obj ========
 *  This structure must be the first field of all G711DEC instance objects.
 */
typedef struct IG711DEC_Obj {
    struct IG711DEC_Fxns *fxns;
} IG711DEC_Obj;

/*
 *  ======== IG711DEC_Handle ========
 *  This handle is used to reference all G711DEC instance objects.
 */
typedef struct IG711DEC_Obj *IG711DEC_Handle;

/*
 *  ======== IG711DEC_Params ========
 *  This structure defines the creation parameters for all G711DEC instance
 *  objects.
 */
typedef struct IG711DEC_Params {
    Int          size;          /* Size of this structure */ 
    XDAS_UInt16  frameLen;      /* Number of samples in a input frame */
    IG711_Mode   mode;          /* Format of the encoded input samples */
} IG711DEC_Params;

extern const IG711DEC_Params IG711DEC_PARAMS;       /* default params */

/*
 *  ======== IG711DEC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG711DEC_Status {
    Int          size;          /* Size of this structure */ 
    XDAS_UInt16  frameLen;      /* Num samples in the in frame (Read/Write) */  
    IG711_Mode   mode;          /* Format of encoded in samples (Read-Only) */
} IG711DEC_Status;

/*
 *  ======== IG711DEC_Fxns ========
 *  This structure defines all of the operations on the G711DEC instance object.
 */
typedef struct IG711DEC_Fxns {
    IALG_Fxns   ialg;    /* IG711DEC extends IALG */
    XDAS_Bool   (*control)(IG711DEC_Handle handle, IG711_Cmd cmd, IG711DEC_Status *status);
    XDAS_Int16 (*decode)(IG711DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);
} IG711DEC_Fxns;

#endif  /* IG711DEC_ */
