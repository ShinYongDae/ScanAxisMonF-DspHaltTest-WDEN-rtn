/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== dtmf.c ========
 *  DTMF detection Module - implements all functions and defines
 *  all constant structures common to all DTMF algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <dtmf.h>

/*
 *  ======== DTMF_apply ========
 *  Apply a DTMF detector to the in buffer. Place the detected events in the
 *  out buffer and return number of events detected.
 */
XDAS_Int8 DTMF_apply(DTMF_Handle handle, XDAS_Int16 *in, XDAS_Int16 *out)
{
    XDAS_Int8  numEvents;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->detect == NULL) {
        return (-1);   /* detection failed */
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Detect digits */
    numEvents=handle->fxns->detect(handle, in, out); 
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numEvents);
}

/*
 *  ======== DTMF_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool DTMF_control(DTMF_Handle handle, DTMF_Cmd cmd, DTMF_Status *status)
{
    /* Check for valid handle and fxn ptr and do control operation */
    if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== DTMF_exit ========
 *  DTMF module finalization
 */
Void DTMF_exit(Void)
{
}

/*
 *  ======== DTMF_init ========
 *  DTMF module initialization
 */
Void DTMF_init(Void)
{
}

