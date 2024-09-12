/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== cptd.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the CPTD concrete interface.
 */
#ifndef CPTD_
#define CPTD_

#include <alg.h>
#include <xdas.h>
#include <icptd.h>

/*
 *  ======== ICPTD_Tone ========
 *  Definition of the possible tones for a CPTD instance object. 
 */
typedef ICPTD_Tone CPTD_Tone;

#define CPTD_DIALTONE      ICPTD_DIALTONE      /* Dial tone */
#define CPTD_RINGINGTONE   ICPTD_RINGINGTONE   /* Ringing tone */
#define CPTD_BUSYTONE      ICPTD_BUSYTONE      /* Busy/Congestion tone */
#define CPTD_SITTONE       ICPTD_SITTONE       /* Special Information tone */
#define CPTD_WARNINGTONE   ICPTD_WARNINGTONE   /* Warning tone (recording) */
#define CPTD_PAYPHONETONE  ICPTD_PAYPHONETONE  /* Payphone recognition tone */
#define CPTD_CALLWTONE     ICPTD_CALLWTONE     /* Call waiting tone */
#define CPTD_CALLERWTONE   ICPTD_CALLERWTONE   /* Caller waiting tone */

/*
 *  ======== CPTD_Cmd ========
 *  CPTD instance control commands.
 */
typedef ICPTD_Cmd CPTD_Cmd;

/* control commands */ 
#define CPTD_GETSTATUS ICPTD_GETSTATUS
#define CPTD_SETSTATUS ICPTD_SETSTATUS

/*
 *  ======== CPTD_Handle ========
 *  This handle is used to reference a CPTD instance object.
 */
typedef struct ICPTD_Obj *CPTD_Handle;

/*
 *  ======== CPTD_Params ========
 *  CPTD instance object creation parameters.
 */
typedef ICPTD_Params CPTD_Params;

/*
 *  ======== CPTD_PARAMS ========
 *  Default CPTD instance object creation parameters.
 */
#define CPTD_PARAMS ICPTD_PARAMS

/*
 *  ======== CPTD_Status ========
 *  CPTD control/status structure.
 */
typedef ICPTD_Status CPTD_Status;

/*
 *  ======== CPTD_apply ========
 *  Apply a CPTD detector to the in buffer. Place the detected tones in the
 *  out buffer and return number of tones detected.
 */
extern XDAS_Int8 CPTD_apply(CPTD_Handle handle, XDAS_Int16 *in, CPTD_Tone *out);

/*
 *  ======== CPTD_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool CPTD_control(CPTD_Handle handle, CPTD_Cmd cmd, CPTD_Status *status);

/*
 *  ======== CPTD_create ========
 *  Create an instance of a CPTD object.
 */
static inline CPTD_Handle CPTD_create(const ICPTD_Fxns *fxns, const CPTD_Params *prms)
{
    return((CPTD_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== CPTD_delete ========
 *  Delete an instance of a CPTD object.
 */
static inline Void CPTD_delete(CPTD_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== CPTD_exit ========
 *  CPTD module finalization
 */
extern Void CPTD_exit(Void);

/*
 *  ======== CPTD_init ========
 *  CPTD module initialization
 */
extern Void CPTD_init(Void);

#endif  /* CPTD_ */
