/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g729enc.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G729ENC concrete interface.
 */
#ifndef G729ENC_
#define G729ENC_

#include <alg.h>
#include <xdas.h>
#include <ig729.h>
#include <ig729enc.h>

/*
 *  ======== G729ENC_Cmd ========
 *  G729ENC instance control commands.
 */
typedef IG729_Cmd G729ENC_Cmd;

/* control commands */ 
#define G729ENC_GETSTATUS IG729_GETSTATUS
#define G729ENC_SETSTATUS IG729_SETSTATUS

/*
 *  ======== G729ENC_Handle ========
 *  This handle is used to reference a G729ENC instance object.
 */
typedef struct IG729ENC_Obj *G729ENC_Handle;

/*
 *  ======== G729ENC_Params ========
 *  G729ENC instance object creation parameters.
 */
typedef IG729ENC_Params G729ENC_Params;
/*
 *  ======== G729ENC_PARAMS ========
 *  Default G729ENC instance object creation parameters.
 */
#define G729ENC_PARAMS IG729ENC_PARAMS

/*
 *  ======== G729ENC_Status ========
 *  G729ENC control/status structure.
 */
typedef IG729ENC_Status G729ENC_Status;

/*
 *  ======== G729ENC_apply ========
 *  Apply a G729ENC encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of bytes in the
 *  out buffer.
 */
extern XDAS_Int8 G729ENC_apply(G729ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);

/*
 *  ======== G729ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G729ENC_control(G729ENC_Handle handle, G729ENC_Cmd cmd, G729ENC_Status *status);

/*
 *  ======== G729ENC_create ========
 *  Create an instance of a G729ENC object.
 */
static inline G729ENC_Handle G729ENC_create(const IG729ENC_Fxns *fxns, const G729ENC_Params *prms)
{
    return((G729ENC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G729ENC_delete ========
 *  Delete an instance of a G729ENC object.
 */
static inline Void G729ENC_delete(G729ENC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G729ENC_exit ========
 *  G729ENC module finalization
 */
extern Void G729ENC_exit(Void);

/*
 *  ======== G729ENC_init ========
 *  G729ENC module initialization
 */
extern Void G729ENC_init(Void);

#endif  /* G729ENC_ */
