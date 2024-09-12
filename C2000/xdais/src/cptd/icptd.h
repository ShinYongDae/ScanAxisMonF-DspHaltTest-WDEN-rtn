/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== icptd.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the abstract interface for the CPTD module.
 */
#ifndef ICPTD_
#define ICPTD_

#include <xdas.h>
#include <ialg.h>

/*
 *  ======== ICPTD_Tone ========
 *  Definition of the possible tones for a CPTD instance object. 
 */
typedef XDAS_Int16 ICPTD_Tone;

#define ICPTD_DIALTONE      ((ICPTD_Tone)-1)   /* Dial tone */
#define ICPTD_RINGINGTONE   ((ICPTD_Tone)-2)   /* Ringing tone */
#define ICPTD_BUSYTONE      ((ICPTD_Tone)-3)   /* Busy/Congestion tone */
#define ICPTD_SITTONE       ((ICPTD_Tone)-4)   /* Special Information tone */
#define ICPTD_WARNINGTONE   ((ICPTD_Tone)-5)   /* Warning tone (recording) */
#define ICPTD_PAYPHONETONE  ((ICPTD_Tone)-6)   /* Payphone recognition tone */
#define ICPTD_CALLWTONE     ((ICPTD_Tone)-7)   /* Call waiting tone */
#define ICPTD_CALLERWTONE   ((ICPTD_Tone)-8)   /* Caller waiting tone */

/*
 *  ======== ICPTD_Obj ========
 *  This structure must be the first field of all CPTD instance objects.
 */
typedef struct ICPTD_Obj {
    struct ICPTD_Fxns *fxns;
} ICPTD_Obj;

/*
 *  ======== ICPTD_Handle ========
 *  This handle is used to reference all CPTD instance objects.
 */
typedef struct ICPTD_Obj *ICPTD_Handle;

/*
 *  ======== ICPTD_Params ========
 *  This structure defines the creation parameters for a CPTD instance object. 
 */
typedef struct ICPTD_Params {
    XDAS_UInt8   size;          /* Size of this structure */ 
    XDAS_UInt16  frameLen;      /* Frame length in number of samples */
} ICPTD_Params;

/*
 *  ======== ICPTD_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct ICPTD_Status {
    XDAS_UInt8   size;        /* Size of this structure */
    ICPTD_Tone   lastTone;    /* Last detected Tone (Read-Only) */
} ICPTD_Status;

/*
 *  ======== ICPTD_Cmd ========
 *  Control commands for a CPTD instance object.
 */
typedef enum ICPTD_Cmd {
    ICPTD_GETSTATUS,
    ICPTD_SETSTATUS
} ICPTD_Cmd;

/*
 *  ======== ICPTD_Fxns ========
 *  This structure defines all of the operations on CPTD objects.
 */
typedef struct ICPTD_Fxns {
    IALG_Fxns   ialg;    /* ICPTD extends IALG */
    XDAS_Bool   (*control)(ICPTD_Handle handle, ICPTD_Cmd cmd, ICPTD_Status *status);
    XDAS_Int8   (*detect)(ICPTD_Handle handle, XDAS_Int16 *in, ICPTD_Tone *out);
} ICPTD_Fxns;

#endif  /* ICPTD_ */
