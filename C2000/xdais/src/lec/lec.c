/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== lec.c ========
 *  LEC Line Echo Canceller Module - implements all functions and defines
 *  all constant structures common to all LEC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <lec.h>

/*
 *  ======== LEC_apply ========
 *  Apply a LEC echo canceller on the nearEndIn buffer and place the result
 *  in the nearEndout buffer. farEndIn is used to update the delay buffer.
 */
XDAS_Bool LEC_apply(LEC_Handle handle, XDAS_Int16 *farEndIn, XDAS_Int16 *nearEndIn, XDAS_Int16 *nearEndOut)
{
    XDAS_Int16    numEchoCancel, numFeedData;

    /* Check for valid handle and fxn ptrs */
    if (handle == NULL || handle->fxns->feedData == NULL \
                       || handle->fxns->echoCancel == NULL) {
       return (XDAS_FALSE);
    }
    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Cancel echo */
    numFeedData = handle->fxns->feedData(handle, farEndIn);
    numEchoCancel = handle->fxns->echoCancel(handle, nearEndIn, nearEndOut);

    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    if (numFeedData < 0 || numEchoCancel < 0) {
        return (XDAS_FALSE);
    }

    return (XDAS_TRUE);
}

/*
 *  ======== LEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool LEC_control(LEC_Handle handle, LEC_Cmd cmd, LEC_Status *status)
{

    if (handle && handle->fxns->control) {
        return (handle->fxns->control(handle, cmd, status));
    }
    
    return (XDAS_FALSE);        /* control not supported */
}

/*
 *  ======== LEC_exit ========
 *  LEC module finalization
 */
Void LEC_exit(Void)
{
}

/*
 *  ======== LEC_init ========
 *  LEC module initialization
 */
Void LEC_init(Void)
{
}

