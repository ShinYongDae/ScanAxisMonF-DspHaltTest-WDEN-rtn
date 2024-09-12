/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g726dec.c ========
 *  G726 decoder module - implements all functions and defines all constants
 *  and structures common to all G726DEC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g726dec.h>
#include <ialg.h>

/*
 *  ======== G726DEC_apply ========
 *  Apply a G726 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
XDAS_Int16 G726DEC_apply(G726DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out)
{
    XDAS_Int16  numSamples;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->decode == NULL ) {
        return (-1);
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Decode */
    numSamples = handle->fxns->decode(handle, in, out);
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numSamples);
}

/*
 *  ======== G726DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G726DEC_control(G726DEC_Handle handle, G726DEC_Cmd cmd, G726DEC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G726DEC_exit ========
 *  G726DEC module finalization
 */
Void G726DEC_exit(Void)
{
}

/*
 *  ======== G726DEC_init ========
 *  G726DEC module initialization
 */
Void G726DEC_init(Void)
{
}

