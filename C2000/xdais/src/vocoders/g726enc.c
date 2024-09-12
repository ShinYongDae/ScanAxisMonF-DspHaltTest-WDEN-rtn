/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g726enc.c ========
 *  G726 encoder module - implements all functions and defines all constants
 *  and structures common to all G726ENC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g726enc.h>
#include <ialg.h>

/*
 *  ======== G726ENC_apply ========
 *  Apply a G726 encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
XDAS_Int16 G726ENC_apply(G726ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out)
{
    XDAS_Int16  numSamples;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->encode == NULL) {
        return (-1);
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Encode */
    numSamples = handle->fxns->encode(handle, in, out);
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numSamples);
}

/*
 *  ======== G726ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G726ENC_control(G726ENC_Handle handle, G726ENC_Cmd cmd, G726ENC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G726ENC_exit ========
 *  G726ENC module finalization
 */
Void G726ENC_exit(Void)
{
}

/*
 *  ======== G726ENC_init ========
 *  G726ENC module initialization
 */
Void G726ENC_init(Void)
{
}

