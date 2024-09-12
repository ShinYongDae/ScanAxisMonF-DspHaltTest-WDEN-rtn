/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ilec.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the abstract interface for the LEC module.
 */
#ifndef ILEC_
#define ILEC_

#include <xdas.h>
#include <ialg.h>

/*
 *  ======== ILEC_Obj ========
 *  This structure must be the first field of all LEC instance objects.
 */
typedef struct ILEC_Obj {
    struct ILEC_Fxns *fxns;
} ILEC_Obj;

/*
 *  ======== ILEC_Handle ========
 *  This handle is used to reference all LEC instance objects.
 */
typedef struct ILEC_Obj *ILEC_Handle;

/*
 *  ======== ILEC_Params ========
 *  This structure defines the creation parameters for a LEC instance object. 
 */
typedef struct ILEC_Params {
    Int          size;          /* Size of this structure */ 
    XDAS_Bool    adaptEnable;   /* Enable/Disable adaptation */
    XDAS_Bool    dTalkEnable;   /* Enable/Disable double talk detector */
    XDAS_UInt16  frameLen;      /* Frame length in number of samples */
    XDAS_Bool    nonLPEnable;   /* Enable/Disable non-linear processor */
    XDAS_UInt16  bulkDelay;     /* Delay in hybrid and system I/O */
    XDAS_UInt16  tailLen;       /* Tail length in number of samples */ 
} ILEC_Params;

extern const ILEC_Params ILEC_PARAMS;   /* default parameters */

/*
 *  ======== ILEC_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct ILEC_Status {
    Int          size;          /* Size of this structure */ 
    XDAS_Bool    adaptEnable;   /* Adaptation on/off (Read/Write) */
    XDAS_Bool    dTalkEnable;   /* Double talk detector on/off (Read/Write) */
    XDAS_Bool    dTalkPresent;  /* Double talk currently present (Read-Only) */
    XDAS_Void    *filterCoeffs; /* Pointer to filter coeffs (Read/Write) */
    XDAS_Bool    nonLPEnable;   /* Non-linear processor on/off (Read/write) */
} ILEC_Status;

/*
 *  ======== ILEC_Cmd ========
 *  Control commands for a LEC instance object.
 */
typedef enum ILEC_Cmd {
    ILEC_GETSTATUS,
    ILEC_SETSTATUS
} ILEC_Cmd;

/*
 *  ======== ILEC_Fxns ========
 *  This structure defines all of the operations on LEC objects.
 */
typedef struct ILEC_Fxns {
    IALG_Fxns  ialg;    /* ILEC extends IALG */
    XDAS_Bool  (*control)(ILEC_Handle handle, ILEC_Cmd cmd, ILEC_Status *status);
    XDAS_Int16 (*echoCancel)(ILEC_Handle handle, XDAS_Int16 *nearEndIn, XDAS_Int16 *nearEndOut);
    XDAS_Int16 (*feedData)(ILEC_Handle handle, XDAS_Int16 *farEndIn);
} ILEC_Fxns;

#endif  /* ILEC_ */
