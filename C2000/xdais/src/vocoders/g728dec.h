/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g728dec.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G728DEC concrete interface.
 */
#ifndef G728DEC_
#define G728DEC_

#include <alg.h>
#include <xdas.h>
#include <ig728.h>
#include <ig728dec.h>

/*
 *  ======== G728DEC_Cmd ========
 *  G728DEC instance control commands.
 */
typedef IG728_Cmd G728DEC_Cmd;

/* control commands */ 
#define G728DEC_GETSTATUS IG728_GETSTATUS
#define G728DEC_SETSTATUS IG728_SETSTATUS

/*
 *  ======== G728DEC_Handle ========
 *  This handle is used to reference a G728DEC instance object.
 */
typedef struct IG728DEC_Obj *G728DEC_Handle;

/*
 *  ======== G728DEC_Mode ========
 *  Instance object working mode.
 */
typedef IG728_Mode G728DEC_Mode;

/* Working modes */
#define G728DEC_LINEAR IG728_LINEAR 
#define G728DEC_ULAW   IG728_ULAW   
#define G728DEC_ALAW   IG728_ALAW   

/*
 *  ======== G728DEC_Params ========
 *  G728DEC instance object creation parameters.
 */
typedef IG728DEC_Params G728DEC_Params;

/*
 *  ======== G728DEC_PARAMS ========
 *  Default G728DEC instance object creation parameters.
 */
#define G728DEC_PARAMS IG728DEC_PARAMS

/*
 *  ======== G728DEC_Status ========
 *  G728DEC control/status structure.
 */
typedef IG728DEC_Status G728DEC_Status;

/*
 *  ======== G728DEC_apply ========
 *  Apply a G728 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
extern XDAS_Int8 G728DEC_apply(G728DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);

/*
 *  ======== G728DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G728DEC_control(G728DEC_Handle handle, G728DEC_Cmd cmd, G728DEC_Status *status);

/*
 *  ======== G728DEC_create ========
 *  Create an instance of a G728DEC object.
 */
static inline G728DEC_Handle G728DEC_create(const IG728DEC_Fxns *fxns, const G728DEC_Params *prms)
{
    return((G728DEC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G728DEC_delete ========
 *  Delete an instance of a G728DEC object.
 */
static inline Void G728DEC_delete(G728DEC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G728DEC_exit ========
 *  G728DEC module finalization
 */
extern Void G728DEC_exit(Void);

/*
 *  ======== G728DEC_init ========
 *  G728DEC module initialization
 */
extern Void G728DEC_init(Void);

#endif  /* G728DEC_ */
