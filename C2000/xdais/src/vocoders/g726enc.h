/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g726enc.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G726ENC concrete interface.
 */
#ifndef G726ENC_
#define G726ENC_

#include <alg.h>
#include <xdas.h>
#include <ig726.h>
#include <ig726enc.h>


/*
 *  ======== G726ENC_Cmd ========
 *  G726ENC instance control commands.
 */
typedef IG726_Cmd G726ENC_Cmd;

/* control commands */ 
#define G726ENC_GETSTATUS IG726_GETSTATUS
#define G726ENC_SETSTATUS IG726_SETSTATUS

/*
 *  ======== G726ENC_Handle ========
 *  This handle is used to reference a G726ENC instance object.
 */
typedef struct IG726ENC_Obj *G726ENC_Handle;


/*
 *  ======== G726ENC_IG726ENC ========
 *  This structure defines all of the operations on G726ENC objects.
 *  IG726ENC extends IALG.
 */
extern IG726ENC_Fxns G726ENC_IG726ENC;


/*
 *  ======== G726ENC_Mode ========
 *  Instance object working mode.
 */
typedef IG726_Mode G726ENC_Mode;

/* Working modes */
#define G726ENC_LINEAR IG726_LINEAR 
#define G726ENC_ULAW   IG726_ULAW   
#define G726ENC_ALAW   IG726_ALAW   

/*
 *  ======== G726ENC_Rate ========
 *  Instance object working rate.
 */
typedef IG726_Rate G726ENC_Rate;

/* Working modes */
#define G726ENC_16KBPS IG726_16KBPS
#define G726ENC_24KBPS IG726_24KBPS
#define G726ENC_32KBPS IG726_32KBPS
#define G726ENC_40KBPS IG726_40KBPS

/*
 *  ======== G726ENC_Params ========
 *  G726ENC instance object creation parameters.
 */
typedef IG726ENC_Params G726ENC_Params;

/*
 *  ======== G726ENC_PARAMS ========
 *  Default G726ENC instance object creation parameters.
 */
#define G726ENC_PARAMS IG726ENC_PARAMS

/*
 *  ======== G726ENC_Status ========
 *  G726ENC control/status structure.
 */
typedef IG726ENC_Status G726ENC_Status;

/*
 *  ======== G726ENC_apply ========
 *  Apply a G726ENC encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
extern XDAS_Int16 G726ENC_apply(G726ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);

/*
 *  ======== G726ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G726ENC_control(G726ENC_Handle handle, G726ENC_Cmd cmd, G726ENC_Status *status);

/*
 *  ======== G726ENC_create ========
 *  Create an instance of a G726ENC object.
 */
static inline G726ENC_Handle G726ENC_create(const IG726ENC_Fxns *fxns, const G726ENC_Params *prms)
{
    return((G726ENC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G726ENC_delete ========
 *  Delete an instance of a G726ENC object.
 */
static inline Void G726ENC_delete(G726ENC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G726ENC_exit ========
 *  G726ENC module finalization
 */
extern Void G726ENC_exit(Void);

/*
 *  ======== G726ENC_init ========
 *  G726ENC module initialization
 */
extern Void G726ENC_init(Void);

#endif  /* G726ENC_ */
