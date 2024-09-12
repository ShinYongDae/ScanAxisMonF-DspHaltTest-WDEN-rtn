/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== lec.h ========
 *  This header defines all types, constants, and functions used by
 *  applications that use the LEC concrete interface.
 */
#ifndef LEC_
#define LEC_

#include <alg.h>
#include <xdas.h>
#include <ilec.h>

/*
 *  ======== LEC_Cmd ========
 *  LEC instance control commands.
 */
typedef ILEC_Cmd LEC_Cmd;

/* control commands */ 
#define LEC_GETSTATUS ILEC_GETSTATUS
#define LEC_SETSTATUS ILEC_SETSTATUS

/*
 *  ======== LEC_Handle ========
 *  This handle is used to reference a LEC instance object.
 */
typedef struct ILEC_Obj *LEC_Handle;

/*
 *  ======== LEC_Params ========
 *  LEC instance object creation parameters.
 */
typedef ILEC_Params LEC_Params;

/*
 *  ======== LEC_PARAMS ========
 *  Default LEC instance object creation parameters.
 */
#define LEC_PARAMS ILEC_PARAMS

/*
 *  ======== LEC_Status ========
 *  LEC control/status structure.
 */
typedef ILEC_Status LEC_Status;

/*
 *  ======== LEC_apply ========
 *  Apply a LEC echo canceller on the nearEndIn buffer and place the result
 *  in the nearEndout buffer. farEndIn is used to update the delay buffer.
 */
extern XDAS_Bool LEC_apply(LEC_Handle handle, XDAS_Int16 *farEndIn, XDAS_Int16 *nearEndIn, XDAS_Int16 *nearEndOut);

/*
 *  ======== LEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
extern XDAS_Bool LEC_control(LEC_Handle handle, LEC_Cmd cmd, LEC_Status *status);

/*
 *  ======== LEC_create ========
 *  Create an instance of a LEC object.
 */
static inline LEC_Handle LEC_create(const ILEC_Fxns *fxns, const LEC_Params *prms)
{
    return((LEC_Handle)ALG_create((IALG_Fxns *)fxns, NULL, (IALG_Params *)prms));
}

/*
 *  ======== LEC_delete ========
 *  Delete an instance of a LEC object.
 */
static inline Void LEC_delete(LEC_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== LEC_exit ========
 *  LEC module finalization
 */
extern Void LEC_exit(Void);

/*
 *  ======== LEC_init ========
 *  LEC module initialization
 */
extern Void LEC_init(Void);

#endif  /* LEC_ */
