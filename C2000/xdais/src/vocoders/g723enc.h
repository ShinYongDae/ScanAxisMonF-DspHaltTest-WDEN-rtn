/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g723enc.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G723ENC concrete interface.
 */
#ifndef G723ENC_
#define G723ENC_

#include <alg.h>
#include <xdas.h>
#include <ig723.h>
#include <ig723enc.h>

/*
 *  ======== G723ENC_Cmd ========
 *  G723ENC instance control commands.
 */
typedef IG723_Cmd G723ENC_Cmd;

/* control commands */ 
#define G723ENC_GETSTATUS IG723_GETSTATUS
#define G723ENC_SETSTATUS IG723_SETSTATUS

/*
 *  ======== G723ENC_Handle ========
 *  This handle is used to reference a G723ENC instance object.
 */
typedef struct IG723ENC_Obj *G723ENC_Handle;

/*
 *  ======== G723ENC_Rate ========
 *  Working rate.
 */
typedef IG723_Rate G723ENC_Rate;

/* Working rates */
#define G723ENC_5300BPS IG723_5300BPS
#define G723ENC_6300BPS IG723_6300BPS

/*
 *  ======== G723ENC_Params ========
 *  G723ENC instance object creation parameters.
 */
typedef IG723ENC_Params G723ENC_Params;

/*
 *  ======== G723ENC_PARAMS ========
 *  Default G723ENC instance object creation parameters.
 */
#define G723ENC_PARAMS IG723ENC_PARAMS

/*
 *  ======== G723ENC_Status ========
 *  G723ENC control/status structure.
 */
typedef IG723ENC_Status G723ENC_Status;

/*
 *  ======== G723ENC_apply ========
 *  Apply a G723ENC encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of 8-bit code
 *  words in the in buffer.
 */
extern XDAS_Int8 G723ENC_apply(G723ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);

/*
 *  ======== G723ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G723ENC_control(G723ENC_Handle handle, G723ENC_Cmd cmd, G723ENC_Status *status);

/*
 *  ======== G723ENC_create ========
 *  Create an instance of a G723ENC object.
 */
static  inline G723ENC_Handle G723ENC_create(const IG723ENC_Fxns *fxns, const G723ENC_Params *prms)
{
    return((G723ENC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}
/*
 *  ======== G723ENC_delete ========
 *  Delete an instance of a G723ENC object.
 */
static inline Void G723ENC_delete(G723ENC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G723ENC_exit ========
 *  G723ENC module finalization
 */
extern Void G723ENC_exit(Void);

/*
 *  ======== G723ENC_init ========
 *  G723ENC module initialization
 */
extern Void G723ENC_init(Void);

#endif  /* G723ENC_ */
