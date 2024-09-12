/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig729enc.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G729 encode algorithm.
 */
#ifndef IG729ENC_
#define IG729ENC_

#include <xdas.h>
#include <ialg.h>
#include <ig729.h>

/*
 *  ======== IG729ENC_Obj ========
 *  This structure must be the first field of all G729ENC instance objects.
 */
typedef struct IG729ENC_Obj {
    struct IG729ENC_Fxns *fxns;
} IG729ENC_Obj;

/*
 *  ======== IG729ENC_Handle ========
 *  This handle is used to reference a G729ENC instance object.
 */
typedef struct IG729ENC_Obj *IG729ENC_Handle;

/*
 *  ======== IG729ENC_Params ========
 *  This structure defines the creation parameters for all G729ENC instance
 *  objects.
 */
typedef struct IG729ENC_Params {
    Int         size;      /* Size of this structure */ 
    XDAS_Bool   annexA;    /* Annex A (reduced encoder complexity) */ 
    XDAS_Bool   annexB;    /* Annex B (silence compression scheme) */ 
    XDAS_Bool   vadEnable; /* Voice activity detector */
} IG729ENC_Params;

extern const IG729ENC_Params IG729ENC_PARAMS;    /* default params */

/*
 *  ======== IG729ENC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG729ENC_Status {
    Int         size;      /* Size of this structure */ 
    XDAS_Bool   annexA;    /* Annex A implementation (Read-Only) */ 
    XDAS_Bool   annexB;    /* Annex B implementation (Read_only) */ 
    XDAS_Bool   vadEnable; /* Voice activity detector (Read/Write) */
} IG729ENC_Status;

/*
 *  ======== IG729ENC_Fxns ========
 *  This structure defines all of the operations on G729ENC objects.
 */
typedef struct IG729ENC_Fxns {
    IALG_Fxns   ialg;
    XDAS_Bool   (*control)(IG729ENC_Handle handle, IG729_Cmd cmd, IG729ENC_Status *status);
    XDAS_Int8   (*encode)(IG729ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);
} IG729ENC_Fxns;

#endif  /* IG729ENC_ */
