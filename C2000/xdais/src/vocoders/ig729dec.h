/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig729dec.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the G729 decode algorithm.
 */
#ifndef IG729DEC_
#define IG729DEC_

#include <xdas.h>
#include <ialg.h>
#include <ig729.h>

/*
 *  ======== IG729DEC_Obj ========
 *  This structure must be the first field of all G729DEC instance objects.
 */
typedef struct IG729DEC_Obj {
    struct IG729DEC_Fxns *fxns;
} IG729DEC_Obj;

/*
 *  ======== IG729DEC_Handle ========
 *  This handle is used to reference all G729DEC instance objects.
 */
typedef struct IG729DEC_Obj *IG729DEC_Handle;

/*
 *  ======== IG729DEC_Params ========
 *  This structure defines the parameters necessary to create an
 *  instance of an G729 decoder object.
 */
typedef struct IG729DEC_Params {
    Int         size;      /* Size of this structure */ 
    XDAS_Bool   annexA;    /* Annex A (reduced complexity decoding) */ 
    XDAS_Bool   annexB;    /* Annex B (Silence Insertion Descriptors frames) */ 
    XDAS_Bool   pfoEnable; /* Post Filter enable */ 
} IG729DEC_Params;

extern const IG729DEC_Params IG729DEC_PARAMS;       /* default params */

/*
 *  ======== IG729DEC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IG729DEC_Status {
    Int         size;      /* Size of this structure */ 
    XDAS_Bool   annexA;    /* Annex A (reduced complexity) (Read-Only) */
    XDAS_Bool   annexB;    /* Annex B (SID frames) (Read-Only) */
    XDAS_Bool   pfoEnable; /* Post Filter on/off (Read/Write) */   
} IG729DEC_Status;

/*
 *  ======== IG729DEC_Fxns ========
 *  This structure defines all of the operations on the G729DEC instance object.
 */
typedef struct IG729DEC_Fxns {
    IALG_Fxns   ialg;
    XDAS_Bool   (*control)(IG729DEC_Handle handle, IG729_Cmd cmd, IG729DEC_Status *status);
    XDAS_Int8   (*decode)(IG729DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out, XDAS_UInt8 packetSize);
} IG729DEC_Fxns;

#endif  /* IG729DEC_ */
