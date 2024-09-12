/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g711dec.c ========
 *  G711 decoder module - implements all functions and defines all constants
 *  and structures common to all G711DEC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g711dec.h>
#include <ialg.h>

/*
 *  ======== G711DEC_apply ========
 *  Apply a G711 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of pcm samples
 *  in the out buffer.
 */
XDAS_Int16 G711DEC_apply(G711DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out)
{
    XDAS_UInt16  numSamples;

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
 *  ======== G711DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G711DEC_control(G711DEC_Handle handle, G711DEC_Cmd cmd, G711DEC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G711DEC_exit ========
 *  G711DEC module finalization
 */
Void G711DEC_exit(Void)
{
}

/*
 *  ======== G711DEC_init ========
 *  G711DEC module initialization
 */
Void G711DEC_init(Void)
{
}

