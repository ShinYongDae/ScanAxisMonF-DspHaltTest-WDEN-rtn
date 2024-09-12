/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g711dec.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G711DEC concrete interface.
 */
#ifndef G711DEC_
#define G711DEC_

#include <alg.h>
#include <xdas.h>
#include <ig711.h>
#include <ig711dec.h>

/*
 *  ======== G711DEC_Cmd ========
 *  G711DEC instance control commands.
 */
typedef IG711_Cmd G711DEC_Cmd;

/* control commands */ 
#define G711DEC_GETSTATUS IG711_GETSTATUS
#define G711DEC_SETSTATUS IG711_SETSTATUS

/*
 *  ======== G711DEC_Handle ========
 *  This handle is used to reference a G711DEC instance object.
 */
typedef struct IG711DEC_Obj *G711DEC_Handle;

/*
 *  ======== G711DEC_Mode ========
 *  G711DEC instance object working modes
 */
typedef IG711_Mode G711DEC_Mode;

/* Working modes */
#define G711DEC_ULAW IG711_ULAW
#define G711DEC_ALAW IG711_ALAW

/*
 *  ======== G711DEC_Params ========
 *  G711DEC instance object creation parameters.
 */
typedef IG711DEC_Params G711DEC_Params;

/*
 *  ======== G711DEC_PARAMS ========
 *  Default G711DEC algorithm instance creation parameters
 */
#define G711DEC_PARAMS IG711DEC_PARAMS

/*
 *  ======== G711DEC_Status ========
 *  G711DEC control/status structure.
 */
typedef IG711DEC_Status G711DEC_Status;

/*
 *  ======== G711DEC_apply ========
 *  Apply a G711 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of PCM samples
 *  in the out buffer.
 */
extern XDAS_Int16 G711DEC_apply(G711DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);

/*
 *  ======== G711DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G711DEC_control(G711DEC_Handle handle, G711DEC_Cmd cmd, G711DEC_Status *status);

/*
 *  ======== G711DEC_create ========
 *  Create an instance of a G711DEC object.
 */
static inline  G711DEC_Handle G711DEC_create(const IG711DEC_Fxns *fxns, const G711DEC_Params *prms)
{
    return((G711DEC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G711DEC_delete ========
 *  Delete an instance of a G711DEC object.
 */
static inline Void G711DEC_delete(G711DEC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G711DEC_exit ========
 *  G711DEC module finalization
 */
extern Void G711DEC_exit(Void);

/*
 *  ======== G711DEC_init ========
 *  G711DEC module initialization
 */
extern Void G711DEC_init(Void);

#endif  /* G711DEC_ */
