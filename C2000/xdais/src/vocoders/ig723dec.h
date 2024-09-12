/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== iG723dec.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G723 decode algorithm.
 */
#ifndef IG723DEC_
#define IG723DEC_

#include <xdas.h>
#include <ialg.h>
#include <ig723.h>

/*
 *  ======== IG723DEC_Obj ========
 *  This structure must be the first field of all G723DEC instance objects.
 */
typedef struct IG723DEC_Obj {
    struct IG723DEC_Fxns *fxns;
} IG723DEC_Obj;

/*
 *  ======== IG723DEC_Handle ========
 *  This handle is used to reference all G723DEC instance objects.
 */
typedef struct IG723DEC_Obj *IG723DEC_Handle;

/*
 *  ======== IG723DEC_Params ========
 *  This structure defines the creation parameters for all G723DEC instance
 *  objects.
 */
typedef struct IG723DEC_Params {
    Int         size;       /* Size of this structure */ 
    XDAS_Bool   annexA;     /* Annex A (Silence Insertion Descriptor frames) */
    XDAS_Bool   pfoEnable;  /* Post Filter enable */   
} IG723DEC_Params;

extern const IG723DEC_Params IG723DEC_PARAMS;   /* default params */

/*
 *  ======== IG723DEC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG723DEC_Status {
    Int         size;       /* Size of this structure */ 
    XDAS_Bool   annexA;     /* Annex A (SID frames) (Read-Only) */
    XDAS_Bool   badFrame;   /* Bad frame indictor (CRC) (Read/Write) */
    XDAS_Bool   pfoEnable;  /* Post Filter enable */   
} IG723DEC_Status;

/*
 *  ======== IG723DEC_Fxns ========
 *  This structure defines all of the operations on G723DEC objects
 */
typedef struct IG723DEC_Fxns {
    IALG_Fxns   ialg;    /* IG723DEC extends IALG */
    XDAS_Bool   (*control)(IG723DEC_Handle handle, IG723_Cmd cmd, IG723DEC_Status *status);
    XDAS_Int8   (*decode)(IG723DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);
} IG723DEC_Fxns;

#endif  /* IG723DEC_ */

