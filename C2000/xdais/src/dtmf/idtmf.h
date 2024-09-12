/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== idtmf.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the abstract interface for the DTMF module.
 */
#ifndef IDTMF_
#define IDTMF_

#include <xdas.h>
#include <ialg.h>

/*
 *  ======== Events ========
 *  Definition of the possible events for a DTMF instance object. 
 */
#define IDTMF_EARLY      (-1)   /* Early detection of a digit */
#define IDTMF_FALSEEARLY (-2)   /* Early detection was false */
#define IDTMF_LEADEDGE   (-3)   /* Leading edge */
#define IDTMF_TRAILEDGE  (-4)   /* Trailing edge */
#define IDTMF_DIGIT      (-5)   /* Digit detected */

/*
 *  ======== Digits ========
 *  Definition of the DTMF digits.
 */
#define IDTMF_0          (0)    /* Digit --0-- detected */
#define IDTMF_1          (1)    /* Digit --1-- detected */
#define IDTMF_2          (2)    /* Digit --2-- detected */
#define IDTMF_3          (3)    /* Digit --3-- detected */
#define IDTMF_4          (4)    /* Digit --4-- detected */
#define IDTMF_5          (5)    /* Digit --5-- detected */
#define IDTMF_6          (6)    /* Digit --6-- detected */
#define IDTMF_7          (7)    /* Digit --7-- detected */
#define IDTMF_8          (8)    /* Digit --8-- detected */
#define IDTMF_9          (9)    /* Digit --9-- detected */
#define IDTMF_A          (10)   /* Digit --A-- detected */
#define IDTMF_B          (11)   /* Digit --B-- detected */
#define IDTMF_C          (12)   /* Digit --C-- detected */
#define IDTMF_D          (13)   /* Digit --D-- detected */
#define IDTMF_STAR       (14)   /* Digit --*-- detected */
#define IDTMF_POND       (15)   /* Digit --#-- detected */

/*
 *  ======== IDTMF_Obj ========
 *  This structure must be the first field of all DTMF instance objects.
 */
typedef struct IDTMF_Obj {
    struct IDTMF_Fxns *fxns;
} IDTMF_Obj;

/*
 *  ======== IDTMF_Handle ========
 *  This handle is used to reference all DTMF instance objects.
 */
typedef struct IDTMF_Obj *IDTMF_Handle;

/*
 *  ======== IDTMF_Params ========
 *  This structure defines the creation parameters for a DTMF instance object. 
 */
typedef struct IDTMF_Params {
    Int          size;          /* Size of this structure */ 
    XDAS_UInt16  frameLen;      /* Frame length in number of samples */
} IDTMF_Params;

/*
 *  ======== IDTMF_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IDTMF_Status {
    Int          size;        /* Size of this structure */
    XDAS_Int16   lastDigit;   /* Last detected digit (Read-Only) */
    XDAS_Int16   lastEvent;   /* Last event (Read-Only) */
} IDTMF_Status;

/*
 *  ======== IDTMF_Cmd ========
 *  Control commands for a DTMF instance object.
 */
typedef enum IDTMF_Cmd {
    IDTMF_GETSTATUS,
    IDTMF_SETSTATUS
} IDTMF_Cmd;

/*
 *  ======== IDTMF_Fxns ========
 *  This structure defines all of the operations on DTMF objects.
 */
typedef struct IDTMF_Fxns {
    IALG_Fxns   ialg;    /* IDTMF extends IALG */
    XDAS_Bool   (*control)(IDTMF_Handle handle, IDTMF_Cmd cmd, IDTMF_Status *status);
    XDAS_Int8   (*detect)(IDTMF_Handle handle, XDAS_Int16 *in, XDAS_Int16 *out);
} IDTMF_Fxns;

#endif  /* IDTMF_ */
