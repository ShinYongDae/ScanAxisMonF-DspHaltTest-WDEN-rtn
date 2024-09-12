/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g728enc.c ========
 *  G728 encoder module - implements all functions and defines all constants
 *  and structures common to all G728ENC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g728enc.h>
#include <ialg.h>

/*
 *  ======== G728ENC_apply ========
 *  Apply a G728 encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of bytes in the
 *  out buffer.
 */
XDAS_Int8 G728ENC_apply(G728ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out)
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
 *  ======== G728ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G728ENC_control(G728ENC_Handle handle, G728ENC_Cmd cmd, G728ENC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G728ENC_exit ========
 *  G728ENC module finalization
 */
Void G728ENC_exit(Void)
{
}

/*
 *  ======== G728ENC_init ========
 *  G728ENC module initialization
 */
Void G728ENC_init(Void)
{
}

