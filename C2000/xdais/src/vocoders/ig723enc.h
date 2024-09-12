/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig723enc.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G723 encode algorithm.
 */
#ifndef IG723ENC_
#define IG723ENC_

#include <xdas.h>
#include <ialg.h>
#include <ig723.h>

/*
 *  ======== IG723ENC_Obj ========
 *  This structure must be the first field of all G723ENC instance objects.
 */
typedef struct IG723ENC_Obj {
    struct IG723ENC_Fxns *fxns;
} IG723ENC_Obj;

/*
 *  ======== IG723ENC_Handle ========
 *  This handle is used to reference all G723ENC instance objects.
 */
typedef struct IG723ENC_Obj *IG723ENC_Handle;

/*
 *  ======== IG723ENC_Params ========
 *  This structure defines the creation parameters for all G723ENC instance
 *  objects.
 */
typedef struct IG723ENC_Params {
    Int         size;          /* Size of this structure */ 
    XDAS_Bool   annexA;        /* Annex A (silence compression scheme) */
    XDAS_Bool   hpfEnable;     /* High Pass Filter enable */   
    IG723_Rate  rate;          /* Working Rate */    
    XDAS_Bool   vadEnable;     /* Voice Activity Detector enable */
} IG723ENC_Params;

extern const IG723ENC_Params IG723ENC_PARAMS;   /* default params */

/*
 *  ======== IG723ENC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG723ENC_Status {
    Int         size;          /* Size of this structure */ 
    XDAS_Bool   annexA;        /* Annex A (Read-Only) */
    XDAS_Bool   hpfEnable;     /* High Pass Filter on/off (Read/Write) */
    IG723_Rate  rate;          /* Working Rate (Read/Write) */
    XDAS_Bool   vadEnable;     /* Voice Activity Detector on/off (Read/Write) */
} IG723ENC_Status;

/*
 *  ======== IG723ENC_Fxns ========
 *  This structure defines all of the operations on G723ENC objects.
 */
typedef struct IG723ENC_Fxns {
    IALG_Fxns   ialg;    /* IG723ENC extends IALG */
    XDAS_Bool  (*control)(IG723ENC_Handle handle, IG723_Cmd cmd, IG723ENC_Status *status);
    XDAS_Int8 (*encode)(IG723ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);
} IG723ENC_Fxns;

#endif  /* IG723ENC_ */
