/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g726dec.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G726DEC concrete interface.
 */
#ifndef G726DEC_
#define G726DEC_

#include <alg.h>
#include <xdas.h>
#include <ig726.h>
#include <ig726dec.h>

/*
 *  ======== G726DEC_Cmd ========
 *  G726DEC instance control commands.
 */
typedef IG726_Cmd G726DEC_Cmd;

/* control commands */ 
#define G726DEC_GETSTATUS IG726_GETSTATUS
#define G726DEC_SETSTATUS IG726_SETSTATUS

/*
 *  ======== G726DEC_Handle ========
 *  This handle is used to reference a G726DEC instance object.
 */
typedef struct IG726DEC_Obj *G726DEC_Handle;


/*
 *  ======== G726DEC_IG726DEC ========
 *  This structure defines all of the operations on G726DEC objects.
 *  IG726DEC extends IALG.
 */
extern IG726DEC_Fxns G726DEC_IG726DEC;


/*
 *  ======== G726DEC_Mode ========
 *  Instance object working mode.
 */
typedef IG726_Mode G726DEC_Mode;

/* Working modes */
#define G726DEC_LINEAR IG726_LINEAR 
#define G726DEC_ULAW   IG726_ULAW   
#define G726DEC_ALAW   IG726_ALAW   

/*
 *  ======== G726DEC_Rate ========
 *  Instance object working rate.
 */
typedef IG726_Rate G726DEC_Rate;

/* Working modes */
#define G726DEC_16KBPS IG726_16KBPS
#define G726DEC_24KBPS IG726_24KBPS
#define G726DEC_32KBPS IG726_32KBPS
#define G726DEC_40KBPS IG726_40KBPS

/*
 *  ======== G726DEC_Params ========
 *  G726DEC instance object creation parameters.
 */
typedef IG726DEC_Params G726DEC_Params;

/*
 *  ======== G726DEC_PARAMS ========
 *  Default G726DEC instance object creation parameters.
 */
#define G726DEC_PARAMS IG726DEC_PARAMS

/*
 *  ======== G726DEC_Status ========
 *  G726DEC control/status structure.
 */
typedef IG726DEC_Status G726DEC_Status;

/*
 *  ======== G726DEC_apply ========
 *  Apply a G726 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
extern XDAS_Int16 G726DEC_apply(G726DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);

/*
 *  ======== G726DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G726DEC_control(G726DEC_Handle handle, G726DEC_Cmd cmd, G726DEC_Status *status);

/*
 *  ======== G726DEC_create ========
 *  Create an instance of a G726DEC object.
 */
static inline G726DEC_Handle G726DEC_create(const IG726DEC_Fxns *fxns, const G726DEC_Params *prms)
{
    return((G726DEC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G726DEC_delete ========
 *  Delete an instance of a G726DEC object.
 */
static inline Void G726DEC_delete(G726DEC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G726DEC_exit ========
 *  G726DEC module finalization
 */
extern Void G726DEC_exit(Void);

/*
 *  ======== G726DEC_init ========
 *  G726DEC module initialization
 */
extern Void G726DEC_init(Void);

#endif  /* G726DEC_ */
