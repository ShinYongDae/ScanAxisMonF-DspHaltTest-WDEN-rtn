/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g729enc.c ========
 *  G729 encoder module - implements all functions and defines all constants
 *  and structures common to all G729ENC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g729enc.h>
#include <ialg.h>

/*
 *  ======== G729ENC_apply ========
 *  Apply a G729 encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of bytes in the
 *  out buffer.
 */
XDAS_Int8 G729ENC_apply(G729ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out)
{
    XDAS_Int8  numBytes;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->encode == NULL) {
        return (-1);
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Encode */
    numBytes = handle->fxns->encode(handle, in, out);
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numBytes);
}

/*
 *  ======== G729ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G729ENC_control(G729ENC_Handle handle, G729ENC_Cmd cmd, G729ENC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G729ENC_exit ========
 *  G729ENC module finalization
 */
Void G729ENC_exit(Void)
{
}

/*
 *  ======== G729ENC_init ========
 *  G729ENC module initialization
 */
Void G729ENC_init(Void)
{
}

