/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g723enc.c ========
 *  G723 encoder module - implements all functions and defines all constants
 *  and structures common to all G723ENC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g723enc.h>
#include <ialg.h>

/*
 *  ======== G723ENC_apply ========
 *  Apply a G723 encoder to the in buffer and place the encoded data in
 *  the out buffer. The function should return the number of 8-bit code
 *  words in the in buffer. 
 */
XDAS_Int8 G723ENC_apply(G723ENC_Handle handle, XDAS_Int16 *in, XDAS_Int8 *out)
{
    XDAS_UInt16  numCodeWords;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->encode == NULL) {
        return (-1);
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Encode */
    numCodeWords = handle->fxns->encode(handle, in, out);
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numCodeWords);
}

/*
 *  ======== G723ENC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G723ENC_control(G723ENC_Handle handle, G723ENC_Cmd cmd, G723ENC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G723ENC_exit ========
 *  G723ENC module finalization
 */
Void G723ENC_exit(Void)
{
}

/*
 *  ======== G723ENC_init ========
 *  G723ENC module initialization
 */
Void G723ENC_init(Void)
{
}

