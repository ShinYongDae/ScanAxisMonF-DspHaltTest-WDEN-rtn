/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g729dec.c ========
 *  G729 decoder module - implements all functions and defines all constants
 *  and structures common to all G729DEC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g729dec.h>
#include <ialg.h>

/*
 *  ======== G729DEC_apply ========
 *  Apply a G729 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer. 'packetSize' contains the number of octets in the in buffer.
 */
XDAS_Int8 G729DEC_apply(G729DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out, XDAS_UInt8 packetSize)
{
    XDAS_Int8  numSamples;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->decode == NULL ) {
        return (-1);
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Decode */
    numSamples = handle->fxns->decode(handle, in, out, packetSize);
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numSamples);
}

/*
 *  ======== G729DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G729DEC_control(G729DEC_Handle handle, G729DEC_Cmd cmd, G729DEC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G729DEC_exit ========
 *  G729DEC module finalization
 */
Void G729DEC_exit(Void)
{
}

/*
 *  ======== G729DEC_init ========
 *  G729DEC module initialization
 */
Void G729DEC_init(Void)
{
}

