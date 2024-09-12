/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g729dec.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G729DEC concrete interface.
 */
#ifndef G729DEC_
#define G729DEC_

#include <alg.h>
#include <xdas.h>
#include <ig729.h>
#include <ig729dec.h>

/*
 *  ======== G729DEC_Cmd ========
 *  G729DEC instance control commands.
 */
typedef IG729_Cmd G729DEC_Cmd;

/* control commands */ 
#define G729DEC_GETSTATUS IG729_GETSTATUS
#define G729DEC_SETSTATUS IG729_SETSTATUS

/*
 *  ======== G729DEC_Handle ========
 *  This handle is used to reference a G729DEC instance object.
 */
typedef struct IG729DEC_Obj *G729DEC_Handle;

/*
 *  ======== G729DEC_Params ========
 *  G729DEC instance object creation parameters.
 */
typedef IG729DEC_Params G729DEC_Params;

/*
 *  ======== G729DEC_PARAMS ========
 *  Default G729DEC instance object creation parameters.
 */
#define G729DEC_PARAMS IG729DEC_PARAMS

/*
 *  ======== G729DEC_Status ========
 *  G729DEC control/status structure.
 */
typedef IG729DEC_Status G729DEC_Status;

/*
 *  ======== G729DEC_apply ========
 *  Apply a G729 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer. 'packetSize' contains the number of octets in the in buffer.
 */
extern XDAS_Int8 G729DEC_apply(G729DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out, XDAS_UInt8 packetSize);

/*
 *  ======== G729DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G729DEC_control(G729DEC_Handle handle, G729DEC_Cmd cmd, G729DEC_Status *status);

/*
 *  ======== G729DEC_create ========
 *  Create an instance of a G729DEC object.
 */
static inline G729DEC_Handle G729DEC_create(const IG729DEC_Fxns *fxns, const G729DEC_Params *prms)
{
    return((G729DEC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G729DEC_delete ========
 *  Delete an instance of a G729DEC object.
 */
static inline Void G729DEC_delete(G729DEC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G729DEC_exit ========
 *  G729DEC module finalization
 */
extern Void G729DEC_exit(Void);

/*
 *  ======== G729DEC_init ========
 *  G729DEC module initialization
 */
extern Void G729DEC_init(Void);

#endif  /* G729DEC_ */
