/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g728enc.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G728ENC concrete interface.
 */
#ifndef G728ENC_
#define G728ENC_

#include <alg.h>
#include <xdas.h>
#include <ig728.h>
#include <ig728enc.h>

/*
 *  ======== G728ENC_Cmd ========
 *  G728ENC instance control commands.
 */
typedef IG728_Cmd G728ENC_Cmd;

/* control commands */ 
#define G728ENC_GETSTATUS IG728_GETSTATUS
#define G728ENC_SETSTATUS IG728_SETSTATUS

/*
 *  ======== G728ENC_Handle ========
 *  This handle is used to reference a G728ENC instance object.
 */
typedef struct IG728ENC_Obj *G728ENC_Handle;

/*
 *  ======== G728ENC_Mode ========
 *  Instance object working mode.
 */
typedef IG728_Mode G728ENC_Mode;

/* Working modes */
#define G728ENC_LINEAR IG728_LINEAR 
#define G728ENC_ULAW   IG728_ULAW   
#define G728ENC_ALAW   IG728_ALAW   

/*
 *  ======== G728ENC_Params ========
 *  G728ENC instance object creation parameters.
 */
typedef IG728ENC_Params G728ENC_Params;

/*
 *  ======== G728ENC_PARAMS ========
 *  Default G728ENC instance object creation parameters.
 */
#define G728ENC_PARAMS IG728ENC_PARAMS

/*
 *  ======== G728ENC_Status ========
 *  G728ENC control/status structure.
 */
typedef IG728ENC_Status G728ENC_Status;

/*
 *  ======== G728ENC_apply ========
 *  Apply a G728ENC encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of bytes in the
 *  out buffer.
 */
extern XDAS_Int8 G728ENC_apply(G728ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);

/*
 *  ======== G728ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G728ENC_control(G728ENC_Handle handle, G728ENC_Cmd cmd, G728ENC_Status *status);

/*
 *  ======== G728ENC_create ========
 *  Create an instance of a G728ENC object.
 */
static inline G728ENC_Handle G728ENC_create(const IG728ENC_Fxns *fxns, const G728ENC_Params *prms)
{
    return((G728ENC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G728ENC_delete ========
 *  Delete an instance of a G728ENC object.
 */
static inline Void G728ENC_delete(G728ENC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G728ENC_exit ========
 *  G728ENC module finalization
 */
extern Void G728ENC_exit(Void);

/*
 *  ======== G728ENC_init ========
 *  G728ENC module initialization
 */
extern Void G728ENC_init(Void);

#endif  /* G728ENC_ */
