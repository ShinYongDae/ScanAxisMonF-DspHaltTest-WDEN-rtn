/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g711enc.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G711ENC algorithm.
 */
#ifndef G711ENC_
#define G711ENC_

#include <alg.h>
#include <xdas.h>
#include <ig711.h>
#include <ig711enc.h>

/*
 *  ======== G711ENC_Cmd ========
 *  G711ENC instance control commands.
 */
typedef IG711_Cmd G711ENC_Cmd;

/* control commands */ 
#define G711ENC_GETSTATUS IG711_GETSTATUS
#define G711ENC_SETSTATUS IG711_SETSTATUS

/*
 *  ======== G711ENC_Handle ========
 *  This handle is used to reference a G711ENC instance object.
 */
typedef struct IG711ENC_Obj *G711ENC_Handle;

/*
 *  ======== G711ENC_Mode ========
 *  G711ENC instance object working modes
 */
typedef IG711_Mode G711ENC_Mode;

/* Working modes */
#define G711ENC_ULAW IG711_ULAW
#define G711ENC_ALAW IG711_ALAW

/*
 *  ======== G711ENC_Params ========
 *  G711ENC instance object creation parameters.
 */
typedef IG711ENC_Params G711ENC_Params;

/*
 *  ======== G711ENC_PARAMS ========
 *  Default G711ENC algorithm instance creation parameters
 */
#define G711ENC_PARAMS IG711ENC_PARAMS

/*
 *  ======== G711ENC_Status ========
 *  G711ENC control/status structure.
 */
typedef IG711ENC_Status G711ENC_Status;

/*
 *  ======== G711ENC_apply =======
 *  Apply a G711ENC algorithm to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
extern XDAS_Int16 G711ENC_apply(G711ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out);

/*
 *  ======== G711ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G711ENC_control(G711ENC_Handle handle, G711ENC_Cmd cmd, G711ENC_Status *status);

/*
 *  ======== G711ENC_create ========
 *  Create an instance of a G711ENC object.
 */
static inline G711ENC_Handle G711ENC_create(const IG711ENC_Fxns *fxns, const G711ENC_Params *prms)
{
    return((G711ENC_Handle)ALG_create((IALG_Fxns *)fxns, NULL,  (IALG_Params *)prms));
}

/*
 *  ======== G711ENC_delete ========
 *  Delete an instance of a G711ENC object.
 */
static inline Void G711ENC_delete(G711ENC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G711ENC_exit ========
 *  G711ENC module finalization
 */
extern Void G711ENC_exit(Void);

/*
 *  ======== G711ENC_init ========
 *  G711ENC module initialization
 */
extern Void G711ENC_init(Void);

#endif  /* G711ENC_ */
