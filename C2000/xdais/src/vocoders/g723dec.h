/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g723dec.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the G723DEC concrete interface.
 */
#ifndef G723DEC_
#define G723DEC_

#include <alg.h>
#include <xdas.h>
#include <ig723.h>
#include <ig723dec.h>

/*
 *  ======== G723DEC_Cmd ========
 *  G723DEC instance control commands.
 */
typedef IG723_Cmd G723DEC_Cmd;

/* control commands */ 
#define G723DEC_GETSTATUS IG723_GETSTATUS
#define G723DEC_SETSTATUS IG723_SETSTATUS

/*
 *  ======== G723DEC_Handle ========
 *  This handle is used to reference a G723DEC instance object.
 */
typedef struct IG723DEC_Obj *G723DEC_Handle;

/* default instance creation parameters */
#define G723DEC_PARAMS IG723DEC_PARAMS

/*
 *  ======== G723DEC_Params ========
 *  G723DEC instance object creation parameters.
 */
typedef IG723DEC_Params G723DEC_Params;

/*
 *  ======== G723DEC_PARAMS ========
 *  Default G723DEC instance object creation parameters.
 */
#define G723DEC_PARAMS IG723DEC_PARAMS

/*
 *  ======== G723DEC_Status ========
 *  G723DEC control/status structure.
 */
typedef IG723DEC_Status G723DEC_Status;

/*
 *  ======== G723DEC_apply ========
 *  Apply a G723 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of 8-bit code
 *  words in the in buffer.
 */
extern XDAS_Int8 G723DEC_apply(G723DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out);

/*
 *  ======== G723DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool G723DEC_control(G723DEC_Handle handle, G723DEC_Cmd cmd, G723DEC_Status *status);

/*
 *  ======== G723DEC_create ========
 *  Create an instance of a G723DEC object.
 */
static inline G723DEC_Handle G723DEC_create(const IG723DEC_Fxns *fxns, const G723DEC_Params *prms)
{
    return((G723DEC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== G723DEC_delete ========
 *  Delete an instance of a G723DEC object.
 */
static inline Void G723DEC_delete(G723DEC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== G723DEC_exit ========
 *  G723DEC module finalization
 */
extern Void G723DEC_exit(Void);

/*
 *  ======== G723DEC_init ========
 *  G723DEC module initialization
 */
extern Void G723DEC_init(Void);

#endif  /* G723DEC_ */
