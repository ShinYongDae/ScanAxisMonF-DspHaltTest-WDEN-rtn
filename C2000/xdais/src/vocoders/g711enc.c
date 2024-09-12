/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g711enc.c ========
 *  G711 encoder module - implements all functions and defines all constant
 *  structures common to all G711ENC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g711enc.h>
#include <ialg.h>

/*
 *  ======== G711ENC_apply ========
 *  Apply a G711 encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of samples in the
 *  out buffer.
 */
XDAS_Int16 G711ENC_apply(G711ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out)
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
 *  ======== G711ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G711ENC_control(G711ENC_Handle handle, G711ENC_Cmd cmd, G711ENC_Status *status)
{
    /* CHeck for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G711ENC_exit ========
 *  G711ENC module finalization
 */
Void G711ENC_exit(Void)
{
}

/*
 *  ======== G711ENC_init ========
 *  G711ENC module initialization
 */
Void G711ENC_init(Void)
{
}

