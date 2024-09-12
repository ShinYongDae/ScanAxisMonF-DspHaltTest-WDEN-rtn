/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== g723dec.c ========
 *  G723 decoder module - implements all functions and defines all constants
 *  and structures common to all G723DEC algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <g723dec.h>
#include <ialg.h>

/*
 *  ======== G723DEC_apply ========
 *  Apply a G723 decoder to the in buffer and place the decoded data in
 *  the out buffer. The function should return the number of 8-bit code
 *  words in the in buffer. 
 */
XDAS_Int8 G723DEC_apply(G723DEC_Handle handle, XDAS_Int8 *in, XDAS_Int16 *out)
{
    XDAS_UInt8  numCodeWords;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->decode == NULL ) {
        return (-1);
    }
    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Decode */
    numCodeWords = handle->fxns->decode(handle, in, out);
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numCodeWords);
}

/*
 *  ======== G723DEC_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool G723DEC_control(G723DEC_Handle handle, G723DEC_Cmd cmd, G723DEC_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== G723DEC_exit ========
 *  G723DEC module finalization
 */
Void G723DEC_exit(Void)
{
}

/*
 *  ======== G723DEC_init ========
 *  G723DEC module initialization
 */
Void G723DEC_init(Void)
{
}

