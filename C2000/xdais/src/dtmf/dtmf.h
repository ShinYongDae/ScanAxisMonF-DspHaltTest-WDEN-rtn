/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== dtmf.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the DTMF concrete interface.
 */
#ifndef DTMF_
#define DTMF_

#include <alg.h>
#include <xdas.h>
#include <idtmf.h>

/*
 *  ======== DTMF_Cmd ========
 *  DTMF instance control commands.
 */
typedef IDTMF_Cmd DTMF_Cmd;

/* control commands */ 
#define DTMF_GETSTATUS IDTMF_GETSTATUS
#define DTMF_SETSTATUS IDTMF_SETSTATUS

/*
 *  ======== Events ========
 *  Definition of the possible events for a DTMF instance object. 
 */
#define DTMF_EARLY      IDTMF_EARLY       /* Early detection of a digit */
#define DTMF_FALSEEARLY IDTMF_FALSEEARLY  /* Early detection was false */
#define DTMF_LEADEDGE   IDTMF_LEADEDGE    /* Leading edge */
#define DTMF_TRAILEDGE  IDTMF_TRAILEDGE   /* Trailing edge */
#define DTMF_DIGIT      IDTMF_DIGIT       /* Digit detected */

/*
 *  ======== Digits ========
 *  Definition of the DTMF digits.
 */
#define DTMF_0          IDTMF_0           /* Digit --0-- detected */
#define DTMF_1          IDTMF_1           /* Digit --1-- detected */
#define DTMF_2          IDTMF_2           /* Digit --2-- detected */
#define DTMF_3          IDTMF_3           /* Digit --3-- detected */
#define DTMF_4          IDTMF_4           /* Digit --4-- detected */
#define DTMF_5          IDTMF_5           /* Digit --5-- detected */
#define DTMF_6          IDTMF_6           /* Digit --6-- detected */
#define DTMF_7          IDTMF_7           /* Digit --7-- detected */
#define DTMF_8          IDTMF_8           /* Digit --8-- detected */
#define DTMF_9          IDTMF_9           /* Digit --9-- detected */
#define DTMF_A          IDTMF_A           /* Digit --A-- detected */
#define DTMF_B          IDTMF_B           /* Digit --B-- detected */
#define DTMF_C          IDTMF_C           /* Digit --C-- detected */
#define DTMF_D          IDTMF_d           /* Digit --D-- detected */
#define DTMF_STAR       IDTMF_STAR        /* Digit --*-- detected */
#define DTMF_POND       IDTMF_POND        /* Digit --#-- detected */

/*
 *  ======== DTMF_Handle ========
 *  This handle is used to reference a DTMF instance object.
 */
typedef struct IDTMF_Obj *DTMF_Handle;

/*
 *  ======== DTMF_Params ========
 *  DTMF instance object creation parameters.
 */
typedef IDTMF_Params DTMF_Params;

/*
 *  ======== DTMF_PARAMS ========
 *  Default DTMF instance object creation parameters.
 */
#define DTMF_PARAMS IDTMF_PARAMS

/*
 *  ======== DTMF_Status ========
 *  DTMF control/status structure.
 */
typedef IDTMF_Status DTMF_Status;

/*
 *  ======== DTMF_apply ========
 *  Apply a DTMF detector to the in buffer. Place the detected events in the
 *  out buffer and return number of events detected.
 */
extern XDAS_Int8 DTMF_apply(DTMF_Handle handle, XDAS_Int16 *in, XDAS_Int16 *out);

/*
 *  ======== DTMF_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool DTMF_control(DTMF_Handle handle, DTMF_Cmd cmd, DTMF_Status *status);

/*
 *  ======== DTMF_create ========
 *  Create an instance of a DTMF object.
 */
static inline  DTMF_Handle DTMF_create(const IDTMF_Fxns *fxns, const DTMF_Params *prms)
{
    return((DTMF_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== DTMF_delete ========
 *  Delete an instance of a DTMF object.
 */
static inline Void DTMF_delete(DTMF_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== DTMF_exit ========
 *  DTMF module finalization
 */
extern Void DTMF_exit(Void);

/*
 *  ======== DTMF_init ========
 *  DTMF module initialization
 */
extern Void DTMF_init(Void);

#endif  /* DTMF_ */
